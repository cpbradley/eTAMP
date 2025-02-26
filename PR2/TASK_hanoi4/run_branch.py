#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import pickle as pk
import time
from etamp.actions import ActionInfo
from etamp.stream import StreamInfo
from utils.pybullet_tools.pr2_primitives import BodyPose, Attach, Detach, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, sdg_sample_grasp_hanoi, sdg_sample_place_hanoi, \
    sdg_sample_arm, \
    sdg_ik_arm_hanoi
from utils.pybullet_tools.utils import set_pose, is_placement_hanoi, \
    get_bodies, connect, get_pose, disconnect, LockRenderer, get_max_limit, WorldSaver
from etamp.progressive3 import solve_progressive2
from etamp.pddlstream.utils import read, get_file_path

from etamp.p_uct2 import PlannerUCT
from etamp.tree_node2 import ExtendedNode
from etamp.env_sk_branch import SkeletonEnv
from build_scenario import Scene_hanoi


def get_fixed(robot, movable):
    rigid = [body for body in get_bodies() if body != robot]
    fixed = [body for body in rigid if body not in movable]
    return fixed


def place_movable(certified):
    placed = []
    for literal in certified:
        if literal[0] == 'not':
            fact = literal[1]
            if fact[0] == 'trajcollision':
                _, b, p = fact[1:]
                set_pose(b, p.pose)
                placed.append(b)
    return placed


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
    c = args[-1]  # objects
    [t] = c.value.body_paths
    distance = t.distance()
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

        with LockRenderer():
            for action in list_exe_action:
                for patom in action.add_effects:
                    if patom.name.lower() == "AtBConf".lower():
                        body_config = patom.args[0].value
                        body_config.assign()
                    elif patom.name.lower() == "AtPose".lower():
                        body_pose = patom.args[1].value
                        body_pose.assign()
                    elif patom.name.lower() == "AtGrasp".lower():
                        body_grasp = patom.args[1].value
                        _arm = patom.args[2].value
                        robot = scn.arm_to_robot[_arm]
                        arm = scn.dict_arm[_arm]
                        attachment = body_grasp.attachment(robot, arm)
                        attachment.assign()

        if cost is False:
            return None

        return 0.1 * np.exp(-cost)

    return fn


def postprocess_plan(scn, exe_plan, teleport=False):
    if exe_plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(exe_plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            robot = scn.arm_to_robot[a]
            arm = scn.dict_arm[a]
            [t] = c.commands
            close_gripper = GripperCommand(robot, arm, 0.1, teleport=teleport)
            attach = Attach(robot, arm, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, r, rp, c = args
            robot = scn.arm_to_robot[a]
            arm = scn.dict_arm[a]
            [t] = c.commands
            gripper_joint = get_gripper_joints(robot, arm)[0]
            position = get_max_limit(robot, gripper_joint)
            open_gripper = GripperCommand(robot, arm, position, teleport=teleport)
            detach = Detach(robot, arm, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        else:
            raise ValueError(name)
        # print(i, name, args, new_commands)
        commands += new_commands
    return commands


def play_commands(commands):
    use_control = False
    if use_control:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)


#######################################################

def get_pddlstream_problem(scn):
    """"""

    domain_pddl = read(get_file_path(__file__, 'pddl/domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'pddl/stream.pddl'))

    init = [('CanPick',)]

    dict_init_pose = {}

    for body in scn.regions:
        pose = BodyPose(body, get_pose(body))
        init += [('IsPose', body, pose)]
        init += [('AtPose', body, pose)]
        dict_init_pose[body] = pose

    for body in scn.movable_bodies:
        pose = BodyPose(body, get_pose(body))
        init += [('IsPose', body, pose)]
        init += [('AtPose', body, pose)]
        init += [('Graspable', body)]
        dict_init_pose[body] = pose

    for body in scn.movable_bodies:
        other_bodies = list(set(scn.movable_bodies + scn.regions) - {body})
        for body2 in other_bodies:
            if is_placement_hanoi(body, body2):
                init += [('On', body, body2)]
                init += [('IsSupport', body, dict_init_pose[body], body2, dict_init_pose[body2])]

    init += [('Smaller', scn.bd_body['disc1'], scn.bd_body['peg1']),
             ('Smaller', scn.bd_body['disc1'], scn.bd_body['peg2']),
             ('Smaller', scn.bd_body['disc1'], scn.bd_body['peg3']),
             ('Smaller', scn.bd_body['disc2'], scn.bd_body['peg1']),
             ('Smaller', scn.bd_body['disc2'], scn.bd_body['peg2']),
             ('Smaller', scn.bd_body['disc2'], scn.bd_body['peg3']),
             ('Smaller', scn.bd_body['disc3'], scn.bd_body['peg1']),
             ('Smaller', scn.bd_body['disc3'], scn.bd_body['peg2']),
             ('Smaller', scn.bd_body['disc3'], scn.bd_body['peg3']),
             ('Smaller', scn.bd_body['disc1'], scn.bd_body['disc2']),
             ('Smaller', scn.bd_body['disc1'], scn.bd_body['disc3']),
             ('Smaller', scn.bd_body['disc2'], scn.bd_body['disc3']),
             ('Clear', scn.bd_body['disc1']),
             ('Clear', scn.bd_body['peg2']),
             ('Clear', scn.bd_body['peg3']),
             ]

    goal = ('and',
            ('On', scn.bd_body['disc3'], scn.bd_body['peg3']),
            ('On', scn.bd_body['disc2'], scn.bd_body['disc3']),
            ('On', scn.bd_body['disc1'], scn.bd_body['disc2']),

            # ('On', scn.bd_body['disc2'], scn.bd_body['disc3']),
            # ('On', scn.bd_body['disc1'], scn.bd_body['peg1']),

            )

    # goal = ('On', scn.bd_body['disc1'], scn.bd_body['peg3'])

    stream_info = {'sample-arm': StreamInfo(seed_gen_fn=sdg_sample_arm(scn)),
                   # 'sample-arm': StreamInfo(seed_gen_fn=sdg_sample_arm(scn),
                   #                          free_generator=True, discrete=True, p1=[0, 1, 2]),
                   # 'sample-grasp': StreamInfo(seed_gen_fn=sdg_sample_grasp_hanoi(scn)),
                   'sample-grasp': StreamInfo(seed_gen_fn=sdg_sample_grasp_hanoi(scn),
                                              free_generator=True, discrete=False, p1=[1], p2=[.2]),
                   # 'sample-place': StreamInfo(seed_gen_fn=sdg_sample_place_hanoi(scn),
                   #                            free_generator=True, discrete=False, p1=[1], p2=[.2]),
                   'sample-place': StreamInfo(seed_gen_fn=sdg_sample_place_hanoi(scn)),
                   'ik-arm-motion-from': StreamInfo(seed_gen_fn=sdg_ik_arm_hanoi(scn)),
                   'ik-arm-motion-to': StreamInfo(seed_gen_fn=sdg_ik_arm_hanoi(scn)),
                   }

    action_info = {'place': ActionInfo(optms_cost_fn=get_const_cost_fn(1), cost_fn=get_const_cost_fn(1)),
                   'pick': ActionInfo(optms_cost_fn=get_const_cost_fn(1), cost_fn=get_const_cost_fn(1)),
                   }

    return domain_pddl, stream_pddl, init, goal, stream_info, action_info


#######################################################

def main(new_problem=1):
    visualization = 1
    connect(use_gui=visualization)

    scn = Scene_hanoi()

    pddlstream_problem = get_pddlstream_problem(scn)
    _, _, _, _, stream_info, action_info = pddlstream_problem

    # pr = cProfile.Profile()
    # pr.enable()

    st = time.time()

    if new_problem:
        sk_batch = solve_progressive2(pddlstream_problem,
                                      num_optms_init=800, target_sk=1)
        op_plan = sk_batch.generate_operatorPlan(0)  # 6
    else:
        with open('temp/C_operatorPlans/C_op_sas.1.pk', 'rb') as f:
            op_plan = pk.load(f)

    e_root = ExtendedNode()
    assert op_plan is not None
    skeleton_env = SkeletonEnv(e_root.num_children, op_plan,
                               get_update_env_reward_fn(scn, action_info),
                               stream_info, scn)
    selected_branch = PlannerUCT(skeleton_env, pw_const=5, ucb_const=0.1)

    concrete_plan = selected_branch.think(500, 0)

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

    # pr.disable()
    # pstats.Stats(pr).sort_stats('tottime').print_stats(10)

    if exe_plan is None:
        disconnect()
        return

    disconnect()
    connect(use_gui=True)
    Scene_hanoi()

    with LockRenderer():
        commands = postprocess_plan(scn, exe_plan)

    play_commands(commands)

    disconnect()

    print('Finished.')


if __name__ == '__main__':
    main()
