#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import cProfile
import pstats
import argparse
import pickle as pk
import time
from etamp.actions import ActionInfo
from etamp.stream import StreamInfo
from utils.pybullet_tools.panda_primitives import sdg_fn, get_stable_gen_table, get_grasp_gen, get_stable_gen_home, \
    get_stable_gen_block, get_ik_fn, get_free_motion_gen, get_holding_motion_gen
from utils.pybullet_tools.kuka_primitives3 import Command, sdg_sample_place, sdg_sample_grasp_dir, \
    sdg_sample_grasp, sdg_ik_grasp, sdg_plan_free_motion, \
    sdg_plan_holding_motion, sdg_sample_stack, Register
from utils.pybullet_tools.utils import WorldSaver, connect, get_pose, set_pose, get_configuration, is_placement, \
    disconnect
import pb_robot
from pb_robot.vobj import BodyPose, BodyConf

from etamp.progressive3 import solve_progressive, solve_progressive2
from etamp.pddlstream.utils import read, INF, get_file_path, find_unique

from etamp.p_uct2 import PlannerUCT
from etamp.tree_node2 import ExtendedNode
from etamp.env_sk_branch import SkeletonEnv
# from .build_scenario import Scene_unpack3,Scene_unpack2,Scene_unpack1,Scene_unpack_pb
from .build_scenario import Scene_unpack_pb


def get_fixed(robot, movable):
    rigid = [body for body in pb_robot.utils.get_bodies() if body.id != robot.id]
    movable_ids = [m.id for m in movable]
    fixed = [body for body in rigid if body.id not in movable_ids]
    return fixed


def postprocess_plan(scn, exe_plan):
    paths = []
    for name, args in exe_plan:
        if name in ['place', 'stack']:
            paths += args[-1].reverse().body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            paths += args[-1].body_paths
        elif name == 'locate_body':
            camera = args[-1]
            body = args[0]
            paths += [Register(camera, body)]
    return Command(paths)


def extract_motion(action_plan):
    """
    Return a list of robot motions
    each of which corresponds to a motor action in the action_plan.
    """
    list_motion = []
    for name, args, _ in action_plan:
        # args are instances of classes
        cmd = args[-1]
        if name == 'place':
            """Since 'place' is the reversed motion of 'pick',
               its path is simply the reverse of what generated by 'pick'."""
            reversed_cmd = cmd.reverse()
            list_motion += reversed_cmd.body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            list_motion += cmd.body_paths
    print('list of paths ----------------------------')
    print(action_plan)
    print(list_motion)
    print('----------------------------------')
    return list_motion


def move_cost_fn(*args):
    """
    :param c: Commands
    """
    cs = args[-1]  # objects
    # [t] = c.value.body_paths
    # [t] = c.value.path
    distance = 0
    for c in cs.value:
        print(c)
        print(type(c))
        distance_fn = pb_robot.planning.get_distance_fn(c.manip, c.manip.joints)
        for q1, q2 in zip(c.path[:-1], c.path[1:]):
            distance += distance_fn(q1, q2)
    # distance = t.distance()
    return distance + 0.1


def get_const_cost_fn(cost):
    def fn(*args):
        return cost

    return fn


def get_action_cost(plan):
    cost = None
    if plan:
        cost = 0
        for paction in plan:
            if callable(paction.pa_info.cost_fn):
                cost += paction.pa_info.cost_fn(*paction.args)
        # print('Action Cost ====================== ', cost)
    return cost


def get_update_env_reward_fn(scn, action_info):
    def get_actions_cost(exe_plan):
        cost = None
        if exe_plan:
            cost = 0
            for action in exe_plan:
                if action.name not in action_info:
                    continue
                cost_fn = action_info[action.name].cost_fn
                if callable(cost_fn):
                    cost += cost_fn(*action.parameters)
        return cost

    def fn(list_exe_action):

        cost = get_actions_cost(list_exe_action)

        """Execution uncertainty will be implemented here."""
        for action in list_exe_action:
            for patom in action.add_effects:
                if patom.name.lower() == "AtConf".lower():
                    body_config = patom.args[0].value
                    # body_config.assign()
                    # body_config.configuration.assign()
                    body_config.manip.arm.SetJointValues(body_config.configuration)
                elif patom.name.lower() == "AtPose".lower():
                    body_pose = patom.args[1].value
                    # body_pose.assign()
                    body_pose.body.set_base_link_pose(body_pose.pose)
                elif patom.name.lower() == "AtGrasp".lower():
                    body_grasp = patom.args[1].value
                    # attachment = body_grasp.attachment()
                    # attachment.assign()

        if cost is False:
            return None

        return 0.1 * np.exp(-cost)

    return fn


def play_commands(commands):
    use_control = False
    if use_control:
        commands.refine(num_steps=10).control(real_time=False, dt=0.0001)
    else:
        commands.refine(num_steps=10).execute(time_step=0.002)


#######################################################

def get_pddlstream_problem(scn):
    # assert (not are_colliding(tree, kin_cache))

    robot = scn.robots[0]
    movable = scn.movable_bodies
    regions = scn.regions

    domain_pddl = read(get_file_path(__file__, 'pddl/domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'pddl/stream.pddl'))

    # conf = BodyConf(robot, robot.get_configuration())
    conf = BodyConf(robot, robot.arm.GetJointValues())
    init = [('CanMove',),
            ('CanPick',),
            ('IsConf', conf),
            ('AtConf', conf),
            ('HandEmpty',),
            ('AllowLocate',), ]

    fixed = get_fixed(robot, movable)

    all_bodies = list(set(movable) | set(fixed))
    for body in movable:
        # pose = BodyPose(body, body.get_pose())
        pose = BodyPose(body, body.get_base_link_pose())
        init += [('Graspable', body),
                 ('IsPose', body, pose),
                 ('AtPose', body, pose)]
        for region in regions:
            init += [('Stackable', body, region)]
            if pb_robot.placements.is_placement(body, region):
                init += [('IsSupport', body, pose, region)]

    for body in regions:
        init += [('Fixed', body)]

    for s in scn.sensors:
        init += [('Fixed', s)]
        init += [('IsSensor', s)]

    goal = ('and',
            ('AtConf', conf),
            ('Holding', scn.bd_body['c1']),
            # ('On', scn.bd_body['c1'], scn.bd_body['region2']),
            )

    # stream_info = {'sample-place': StreamInfo(seed_gen_fn=sdg_sample_place(all_bodies), every_layer=15,
    #                                           free_generator=True, discrete=False, p1=[1, 1, 1], p2=[.2, .2, .2]),
    #                'sample-stack': StreamInfo(seed_gen_fn=sdg_sample_stack(all_bodies), every_layer=15,
    #                                           free_generator=True, discrete=False, p1=[1, 1, 1], p2=[.2, .2, .2]),
    #                'sample-grasp-dir': StreamInfo(seed_gen_fn=sdg_sample_grasp_dir(robot, scn.dic_body_info),
    #                                               every_layer=15,
    #                                               free_generator=True, discrete=True,
    #                                               p1=[2],
    #                                               p2=[10]),
    #             #    'sample-grasp': StreamInfo(seed_gen_fn=sdg_sample_grasp(robot, scn.dic_body_info)),
    #                'sample-grasp': StreamInfo(seed_gen_fn=sdg_sample_grasp(robot, scn.dic_body_info),
    #                                           every_layer=15,
    #                                           free_generator=True, discrete=True,
    #                                           p1=[0, 1, 2, 3],
    #                                           # p1=[0],
    #                                           p2=[4, 4, 4, 4]),
    #                'inverse-kinematics': StreamInfo(seed_gen_fn=sdg_ik_grasp(robot, scn.all_bodies), max_queries=1),
    #                'plan-free-motion': StreamInfo(seed_gen_fn=sdg_plan_free_motion(robot, all_bodies), max_queries=1),
    #                'plan-holding-motion': StreamInfo(seed_gen_fn=sdg_plan_holding_motion(robot, all_bodies), max_queries=1),
    #                }
    # stream_info = {
    #     'sample-pose-table': StreamInfo(seed_gen_fn=from_gen_fn(primitives.get_stable_gen_table(fixed)),
    #     'sample-pose-block': from_fn(primitives.get_stable_gen_block(fixed)),
    #     'sample-grasp': from_gen_fn(primitives.get_grasp_gen(robot)),
    #     'inverse-kinematics': from_fn(primitives.get_ik_fn(robot, fixed)), 
    #     'plan-free-motion': from_fn(primitives.get_free_motion_gen(robot, fixed)),
    #     'plan-holding-motion': from_fn(primitives.get_holding_motion_gen(robot, fixed)),
    # }
    import tamp
    stream_info = {'sample-place': StreamInfo(seed_gen_fn=sdg_fn(get_stable_gen_table, all_bodies), every_layer=15,
                                              free_generator=True, discrete=False, p1=[1, 1, 1], p2=[.2, .2, .2]),
                   'sample-stack': StreamInfo(seed_gen_fn=sdg_fn(get_stable_gen_table, all_bodies), every_layer=15,
                                              free_generator=True, discrete=False, p1=[1, 1, 1], p2=[.2, .2, .2]),
                #    'sample-grasp-dir': StreamInfo(seed_gen_fn=sdg_sample_grasp_dir(robot, scn.dic_body_info),
                #                                   every_layer=15,
                #                                   free_generator=True, discrete=True,
                #                                   p1=[2],
                #                                   p2=[10]),
                #    'sample-grasp': StreamInfo(seed_gen_fn=sdg_sample_grasp(robot, scn.dic_body_info)),
                   'sample-grasp': StreamInfo(seed_gen_fn=sdg_fn(get_grasp_gen, robot),
                                              every_layer=15,
                                              free_generator=True, discrete=True,
                                              p1=[0, 1, 2, 3],
                                              # p1=[0],
                                              p2=[4, 4, 4, 4]),
                   'inverse-kinematics': StreamInfo(seed_gen_fn=sdg_fn(get_ik_fn, robot, fixed), max_queries=1),
                   'plan-free-motion': StreamInfo(seed_gen_fn=sdg_fn(get_free_motion_gen, robot, fixed), max_queries=1),
                   'plan-holding-motion': StreamInfo(seed_gen_fn=sdg_fn(get_holding_motion_gen, robot, fixed), max_queries=1),
                   }
    action_info = {'move_free': ActionInfo(optms_cost_fn=get_const_cost_fn(5), cost_fn=move_cost_fn),
                   'move_holding': ActionInfo(optms_cost_fn=get_const_cost_fn(5), cost_fn=move_cost_fn),
                   'place': ActionInfo(optms_cost_fn=get_const_cost_fn(1), cost_fn=get_const_cost_fn(1)),
                   'pick': ActionInfo(optms_cost_fn=get_const_cost_fn(1), cost_fn=get_const_cost_fn(1)),
                   }

    return domain_pddl, stream_pddl, init, goal, stream_info, action_info


#######################################################

def main(display=True, teleport=False):
    visualization = True

    connect(use_gui=visualization)

    scn = Scene_unpack_pb()

    saved_world = WorldSaver()
    # dump_world()

    pddlstream_problem = get_pddlstream_problem(scn)
    _, _, _, _, stream_info, action_info = pddlstream_problem

    st = time.time()

    new_problem = 1
    if new_problem:
        sk_batch = solve_progressive2(pddlstream_problem,
                                      num_optms_init=80, target_sk=50)
        op_plan = sk_batch.generate_operatorPlan(40)  # c1-39
    else:
        with open('temp/C_operatorPlans/C_op_sas.1.pk', 'rb') as f:
            op_plan = pk.load(f)

    e_root = ExtendedNode()
    assert op_plan is not None
    skeleton_env = SkeletonEnv(e_root.num_children, op_plan,
                               get_update_env_reward_fn(scn, action_info),
                               stream_info, scn)
    selected_branch = PlannerUCT(skeleton_env)

    concrete_plan = selected_branch.think(900, 0)

    if concrete_plan is None:
        print('TAMP is failed.', concrete_plan)
        disconnect()
        return
    thinking_time = time.time() - st
    print('TAMP is successful. think_time: '.format(thinking_time))

    exe_plan = None
    if concrete_plan is not None:
        exe_plan = []
    for action in concrete_plan:
        exe_plan.append((action.name, [arg.value for arg in action.parameters]))

    with open('exe_plan.pk', 'wb') as f:
        pk.dump((scn, exe_plan), f)

    if (not display) or (exe_plan is None):
        disconnect()
        return

    if not visualization:  # TODO: how to reenable the viewer
        disconnect()
        connect(use_gui=True)
        Scene_unpack_pb()
    else:
        saved_world.restore()

    # list_motion = [BodyPath(0,7,51,0), BodyP    action_cost = get_action_cost(action_plan)ath(0,7,8,0), Attach(0,4), ...]
    commands = postprocess_plan(scn, exe_plan)

    play_commands(commands)

    disconnect()
    print('Finished.')


if __name__ == '__main__':
    main()
