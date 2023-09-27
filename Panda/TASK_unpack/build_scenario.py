#!/usr/bin/env python

from __future__ import print_function
import numpy as np

import time
from utils.pybullet_tools.kuka_primitives3 import BodyPose, BodyConf, Register
# from utils.pybullet_tools.utils import WorldSaver, connect, dump_world, get_pose, set_pose, Pose, \
#     Point, set_default_camera, stable_z, disconnect, get_bodies, HideOutput, \
#     create_box, \
#     load_pybullet, step_simulation, Euler, get_links, get_link_info, get_movable_joints, set_joint_positions, \
#     set_camera, get_center_extent, tform_from_pose, attach_viewcone, LockRenderer, get_aabb

import pb_robot

from copy import copy

from tamp.misc import setup_panda_world, get_pddl_block_lookup
                    #   get_pddlstream_info, print_planning_problem, \
                    #   ExecuteActions, ExecutionFailure, get_ik_stream


# class Scene_unpack1(object):
#     def __init__(self):
#         with HideOutput():
#             with LockRenderer():
#                 self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
#                                               fixed_base=True)
#                 self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

#                 self.bd_body = {
#                     'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
#                     'cabinet_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
#                         fixed_base=True),
#                     'drawer_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
#                         fixed_base=True),
#                     'pegboard': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
#                         fixed_base=True),
#                     # 'region1': load_pybullet("../scenario_description/region.urdf", fixed_base=True),
#                     'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
#                     'region2': load_pybullet("../scenario_description/region_big.urdf",
#                                              fixed_base=True),
#                     'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
#                 }
#                 self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

#                 self.drawer_links = get_links(self.bd_body['drawer_shelf'])
#                 cabinet_links = get_links(self.bd_body['cabinet_shelf'])

#                 set_pose(self.bd_body['cabinet_shelf'],
#                          Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['drawer_shelf'],
#                          Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['pegboard'],
#                          Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region1'],
#                         #  Pose(Point(x=0.35, y=0.9, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
#                          Pose(Point(x=0.45, y=0.7, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region2'],
#                          Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

#                 self.movable_bodies = [self.bd_body['c1'], ]
#                 self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
#                                    self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
#                 self.regions = [self.bd_body['region1'], self.bd_body['region2']]

#                 self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

#                 self.sensors = []

#                 self.robots = [self.arm_left]

#                 self.dic_body_info = {}
#                 for b in self.movable_bodies:
#                     obj_center, obj_extent = get_center_extent(b)
#                     body_pose = get_pose(b)
#                     body_frame = tform_from_pose(body_pose)
#                     bottom_center = copy(obj_center)
#                     bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
#                     bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
#                     relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
#                     center_frame = tform_from_pose((obj_center, body_pose[1]))
#                     relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

#                     self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

#                 self.init_poses = sample_block_poses(self.bd_body, n=1)
#                 self.reset()

#     def reset(self):
#         with HideOutput():
#             with LockRenderer():
#                 # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
#                 initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
#                 config_left = BodyConf(self.arm_left, initial_jts)
#                 config_left.assign()

#                 movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
#                 set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

#                 # set_pose(self.bd_body['c1'],
#                 #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
#                 for key, value in self.init_poses.items():
#                     set_pose(self.bd_body[key],
#                             Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))


#                 set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))

#     def get_elemetns(self):
#         self.reset()
#         return self.arm_left, self.movable_bodies, self.regions

# class Scene_unpack2(object):
#     def __init__(self):
#         with HideOutput():
#             with LockRenderer():
#                 self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
#                                               fixed_base=True)
#                 self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

#                 self.bd_body = {
#                     'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
#                     'cabinet_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
#                         fixed_base=True),
#                     'drawer_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
#                         fixed_base=True),
#                     'pegboard': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
#                         fixed_base=True),
#                     # 'region1': load_pybullet("../scenario_description/region.urdf", fixed_base=True),
#                     'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
#                     'region2': load_pybullet("../scenario_description/region_big.urdf",
#                                              fixed_base=True),
#                     'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
#                     'c2': load_pybullet("../scenario_description/boxC.urdf", fixed_base=False),
#                 }
#                 self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

#                 self.drawer_links = get_links(self.bd_body['drawer_shelf'])
#                 cabinet_links = get_links(self.bd_body['cabinet_shelf'])

#                 set_pose(self.bd_body['cabinet_shelf'],
#                          Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['drawer_shelf'],
#                          Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['pegboard'],
#                          Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region1'],
#                         #  Pose(Point(x=0.35, y=0.9, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
#                          Pose(Point(x=0.45, y=0.7, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region2'],
#                          Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

#                 self.movable_bodies = [self.bd_body['c1'], self.bd_body['c2'], ]
#                 self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
#                                    self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
#                 self.regions = [self.bd_body['region1'], self.bd_body['region2']]

#                 self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

#                 self.sensors = []

#                 self.robots = [self.arm_left]

#                 self.dic_body_info = {}
#                 for b in self.movable_bodies:
#                     obj_center, obj_extent = get_center_extent(b)
#                     body_pose = get_pose(b)
#                     body_frame = tform_from_pose(body_pose)
#                     bottom_center = copy(obj_center)
#                     bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
#                     bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
#                     relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
#                     center_frame = tform_from_pose((obj_center, body_pose[1]))
#                     relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

#                     self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

#                 self.init_poses = sample_block_poses(self.bd_body, n=2)
#                 self.reset()

#     def reset(self):
#         with HideOutput():
#             with LockRenderer():
#                 # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
#                 initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
#                 config_left = BodyConf(self.arm_left, initial_jts)
#                 config_left.assign()

#                 movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
#                 set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

#                 # set_pose(self.bd_body['c1'],
#                 #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
#                 # set_pose(self.bd_body['c2'],
#                 #          Pose(Point(x=0.32, y=0.9, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
#                 for key, value in self.init_poses.items():
#                     set_pose(self.bd_body[key],
#                             Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))


#                 set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))

#     def get_elemetns(self):
#         self.reset()
#         return self.arm_left, self.movable_bodies, self.regions


# class Scene_unpack3(object):
#     def __init__(self):
#         import pb_robot
#         # from pb_robot.block_utils import sample_block_poses
#         from latent_feasibility.agents import PandaAgent
#         with HideOutput():
#             with LockRenderer():
#                 self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
#                                               fixed_base=True)
#                 self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

#                 self.bd_body = {
#                     'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
#                     'cabinet_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
#                         fixed_base=True),
#                     'drawer_shelf': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
#                         fixed_base=True),
#                     'pegboard': load_pybullet(
#                         "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
#                         fixed_base=True),
#                     'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
#                     'region2': load_pybullet("../scenario_description/region_big.urdf",
#                                              fixed_base=True),
#                     'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
#                     'c2': load_pybullet("../scenario_description/boxC.urdf", fixed_base=False),
#                     'c3': load_pybullet("../scenario_description/boxCx.urdf", fixed_base=False),
#                 }
#                 self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

#                 self.drawer_links = get_links(self.bd_body['drawer_shelf'])
#                 cabinet_links = get_links(self.bd_body['cabinet_shelf'])

#                 set_pose(self.bd_body['cabinet_shelf'],
#                          Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['drawer_shelf'],
#                          Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['pegboard'],
#                          Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region1'],
#                          Pose(Point(x=0.45, y=0.8, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
#                 set_pose(self.bd_body['region2'],
#                          Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

#                 self.movable_bodies = [self.bd_body['c1'], self.bd_body['c2'], self.bd_body['c3']]
#                 self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
#                                    self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
#                 self.regions = [self.bd_body['region1'], self.bd_body['region2']]

#                 self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

#                 self.sensors = []

#                 self.robots = [self.arm_left]

#                 self.dic_body_info = {}
#                 for b in self.movable_bodies:
#                     obj_center, obj_extent = get_center_extent(b)
#                     body_pose = get_pose(b)
#                     body_frame = tform_from_pose(body_pose)
#                     bottom_center = copy(obj_center)
#                     bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
#                     bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
#                     relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
#                     center_frame = tform_from_pose((obj_center, body_pose[1]))
#                     relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

#                     self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

#                 self.init_poses = sample_block_poses(self.bd_body)
#                 # print(self.init_poses)
#                 # assert False
#                 self.reset()

#     def reset(self):
#         with HideOutput():
#             with LockRenderer():
#                 # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
#                 initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
#                 config_left = BodyConf(self.arm_left, initial_jts)
#                 config_left.assign()

#                 movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
#                 set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

#                 # set_pose(self.bd_body['c1'],
#                 #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
#                 # set_pose(self.bd_body['c2'],
#                 #          Pose(Point(x=0.32, y=0.9, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
#                 #         #  Pose(Point(x=0.02, y=0.7, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
#                 # set_pose(self.bd_body['c3'],
#                 #          Pose(Point(x=0.07, y=0.845, z=stable_z(self.bd_body['c3'], self.bd_body['region1']))))

#                 for key, value in self.init_poses.items():
#                     set_pose(self.bd_body[key],
#                             Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))

#                 set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))
    
        
#     def get_elemetns(self):
#         self.reset()
#         return self.arm_left, self.movable_bodies, self.regions


class Scene_unpack_pb(object):
    def __init__(self):
        # with pb_robot.helper.HideOutput():
        #     with pb_robot.utils.LockRenderer():
                # self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
                #                               fixed_base=True)
                # self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

                # self.bd_body = {
                #     'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
                #     'cabinet_shelf': load_pybullet(
                #         "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
                #         fixed_base=True),
                #     'drawer_shelf': load_pybullet(
                #         "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
                #         fixed_base=True),
                #     'pegboard': load_pybullet(
                #         "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
                #         fixed_base=True),
                #     'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
                #     'region2': load_pybullet("../scenario_description/region_big.urdf",
                #                              fixed_base=True),
                #     'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
                #     'c2': load_pybullet("../scenario_description/boxC.urdf", fixed_base=False),
                #     'c3': load_pybullet("../scenario_description/boxCx.urdf", fixed_base=False),
                # }
                # self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

                # self.drawer_links = get_links(self.bd_body['drawer_shelf'])
                # cabinet_links = get_links(self.bd_body['cabinet_shelf'])

                # set_pose(self.bd_body['cabinet_shelf'],
                #          Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
                # set_pose(self.bd_body['drawer_shelf'],
                #          Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
                # set_pose(self.bd_body['pegboard'],
                #          Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
                # set_pose(self.bd_body['region1'],
                #          Pose(Point(x=0.45, y=0.8, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                # set_pose(self.bd_body['region2'],
                #          Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

                # self.movable_bodies = [self.bd_body['c1'], self.bd_body['c2'], self.bd_body['c3']]
                # self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
                #                    self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
                # self.regions = [self.bd_body['region1'], self.bd_body['region2']]

                # self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

                # self.sensors = []

                # self.robots = [self.arm_left]

                # self.dic_body_info = {}
                # for b in self.movable_bodies:
                #     obj_center, obj_extent = get_center_extent(b)
                #     body_pose = get_pose(b)
                #     body_frame = tform_from_pose(body_pose)
                #     bottom_center = copy(obj_center)
                #     bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
                #     bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
                #     relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
                #     center_frame = tform_from_pose((obj_center, body_pose[1]))
                #     relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

                #     self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)


        # from agents.panda_agent import PandaAgent
        from tamp.misc import load_blocks, load_eval_block
        from block_utils import get_adversarial_blocks, rotation_group, ZERO_POS, \
                Quaternion, get_rotated_block, Pose, add_noise, \
                Environment, Position, World

        # train_blocks_fname = '/catkin_ws/src/latent_feasibility/learning/domains/towers/train_sim_block_set_10.pkl'
        train_blocks_fname = '/catkin_ws/src/latent_feasibility/learning/domains/towers/grasping_block_set_robot.pkl'
        # train_blocks_fname = '/catkin_ws/src/latent_feasibility/learning/domains/towers/final_block_set_10.pkl'
        
        # import os
        # cwd = os.getcwd()
        # os.chdir('/catkin_ws/src/latent_feasibility')
        block_id = 16
        # blocks = load_blocks(train_blocks_fname=train_blocks_fname)
        blocks = load_eval_block(
            blocks_fname=train_blocks_fname,
            eval_block_id=block_id)
        # os.chdir('/catkin_ws/src/latent_feasibility')
        # print(blocks)
        # assert False
        # import pickle
        # # train_blocks_fname = '/catkin_ws/src/latent_feasibility/learning/domains/towers/grasping_block_set_sim.pkl'
        # eval_blocks_fname = ''
        # eval_blocks_ixs=[]
        # remove_ixs=[]
        # num_blocks=3
        # with open(train_blocks_fname, 'rb') as handle:
        #     blocks = pickle.load(handle)
        #     n_train = len(blocks)
        # if len(eval_blocks_fname) > 0:
        #     with open(eval_blocks_fname, 'rb') as handle:
        #         eval_blocks = pickle.load(handle)
        #     for ix in eval_block_ixs:
        #         eval_blocks[ix].name = 'obj_' + str(n_train + 0)
        #         blocks.append(eval_blocks[ix])

        # for ix in sorted(remove_ixs, reverse=True):
        #     del blocks[ix]
        # print(blocks)
        # planning_blocks = blocks[:num_blocks]
        print(blocks)
        # self.init_poses = sample_block_poses(self.bd_body)
        # agent = PandaAgent(blocks)
        agent = PandaAgent(
            blocks=blocks,
            block_init_xy_poses=[
                Pose(
                    Position(0.4, 0.0, 0.0),
                    Quaternion(0.0, 0.0, 0.0, 1.0)
                )
            ]
        )
        self.panda_agent = agent
        self.arm_left = agent.robot
        self.robots = [self.arm_left]
        self.movable_bodies = self.panda_agent.pddl_blocks
        self.regions = [self.panda_agent.table]
        self.sensors = []
        # os.chdir(cwd)
        self.bd_body = {
            'region1': self.regions[0],
            'region2': self.regions[0],
            'c1': self.movable_bodies[0],
            'c2': self.movable_bodies[0],
            'c3': self.movable_bodies[0]
        }
        self.reset()

    def reset(self):

        self.panda_agent.reset_world()
        # with pb_robot.helper.HideOutput():
        #     with pb_robot.utils.LockRenderer():
        #         # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
        #         # initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
        #         # config_left = BodyConf(self.arm_left, initial_jts)
        #         # config_left.assign()

        #         # movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
        #         # set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

        #         # set_pose(self.bd_body['c1'],
        #         #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
        #         # set_pose(self.bd_body['c2'],
        #         #          Pose(Point(x=0.32, y=0.9, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
        #         #         #  Pose(Point(x=0.02, y=0.7, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
        #         # set_pose(self.bd_body['c3'],
        #         #          Pose(Point(x=0.07, y=0.845, z=stable_z(self.bd_body['c3'], self.bd_body['region1']))))

        #         for key, value in self.init_poses.items():
        #             pb_robot.body.set_pose(self.bd_body[key],
        #                     pb_robot.geometry.Pose(pb_robot.geometry.Point(x=value[0], y=value[1], z=pb_robot.placements.stable_z(self.bd_body[key], self.bd_body['region1']))))

        #         pb_robot.utils.set_camera(150, -35, 1.6, pb_robot.geometry.Point(-0.1, 0.1, -0.1))
    
        
    def get_elemetns(self):
        self.reset()
        return self.arm_left, self.movable_bodies, self.regions

class PandaAgent(object):
    def __init__(self, blocks, noise=0.00005, block_init_xy_poses=None,
                teleport=False, use_platform=False, use_vision=False, real=False,
                use_action_server=False, use_learning_server=False):
        """
        Build the Panda world in PyBullet and set up the PDDLStream solver.
        The Panda world should in include the given blocks as well as a
        platform which can be used in experimentation.
        :param teleport: Debugging parameter used to skip planning while moving
                         blocks around this world.
        :param use_platform: Boolean stating whether to include the platform to
                             push blocks off of or not.
        :param use_vision: Boolean stating whether to use vision to detect blocks.
        :param use_action_server: Boolean stating whether to use the separate
                                  ROS action server to do planning.
        :param use_learning_server: Boolean stating whether to host a ROS service
                                    server to drive planning from active learning script.

        If you are using the ROS action server, you must start it in a separate terminal:
            rosrun stacking_ros planning_server.py
        """
        self.real = real
        self.use_vision = use_vision
        self.use_platform = use_platform
        self.use_action_server = use_action_server
        self.use_learning_server = use_learning_server
        self.rs = None

        self.Q_VIEW = [0.00906281211377156, -0.6235522351178657, -0.7049609841296547, -2.688986561330677, 0.16884370372653495, 2.34366196881325, -0.8102285046842361]

        # Setup PyBullet instance to run in the background and handle planning/collision checking.
        self._planning_client_id = pb_robot.utils.connect(use_gui=False)
        self.plan()
        pb_robot.utils.set_default_camera()
        self.robot = pb_robot.panda.Panda()
        self.robot.arm.hand.Open()
        self.belief_blocks = blocks

        self.pddl_blocks, self.platform_table, self.platform_leg, self.table, self.frame, self.wall = setup_panda_world(
            self.robot,
            blocks,
            block_init_xy_poses,
            use_platform=use_platform
        )
        self.fixed = [self.platform_table, self.platform_leg, self.table, self.frame, self.wall]
        self.pddl_block_lookup = get_pddl_block_lookup(blocks, self.pddl_blocks)

        self.orig_joint_angles = self.robot.arm.GetJointValues()
        self.orig_block_poses = [b.get_base_link_pose() for b in self.pddl_blocks]

        # Setup PyBullet instance that only visualizes plan execution. State needs to match the planning instance.
        import block_utils
        poses = [b.get_base_link_pose() for b in self.pddl_blocks]
        poses = [block_utils.Pose(block_utils.Position(*p[0]), block_utils.Quaternion(*p[1])) for p in poses]
        self._execution_client_id = pb_robot.utils.connect(use_gui=True)
        self.execute()
        pb_robot.utils.set_default_camera()
        self.execution_robot = pb_robot.panda.Panda()
        self.execution_robot.arm.hand.Open()
        setup_panda_world(
            self.execution_robot,
            blocks,
            poses,
            use_platform=use_platform
        )

        # Set up ROS plumbing if using features that require it
        if self.use_vision or self.use_action_server or real:
            import rospy
            try:
                rospy.init_node("panda_agent")
            except:
                print("ROS Node already created")

        # Create an arm interface
        if real:
            print('[PandaAgent] Loading franka_interface...')
            from franka_interface import ArmInterface
            self.real_arm = ArmInterface()

            from franka_core_msgs.msg import RobotState
            state_topic = "/franka_ros_interface/custom_franka_state_controller/robot_state"
            self.arm_last_error_time = time.time()
            self.arm_error_check_time = 3.0
            self.arm_state_subscriber = rospy.Subscriber(
                state_topic, RobotState, self.robot_state_callback)
            print('[PandaAgent] Loaded franka_interface.')

        # Setup vision ROS services. Assume cameras can't see blocks initially.
        #TODO: (ICRA) Will probably only be using the wrist camera.
        if self.use_vision:
            from panda_vision.srv import GetBlockPosesWorld, GetBlockPosesWrist
            rospy.wait_for_service('get_block_poses_world')
            rospy.wait_for_service('get_block_poses_wrist')
            self._get_block_poses_world = rospy.ServiceProxy('get_block_poses_world', GetBlockPosesWorld)
            self._get_block_poses_wrist = rospy.ServiceProxy('get_block_poses_wrist', GetBlockPosesWrist)

        # Start ROS clients and servers as needed
        self.last_obj_held = None
        if self.use_action_server:
            raise NotImplementedError('Not updaterd for grasping...')
            from stacking_ros.srv import GetPlan, SetPlanningState, PlanTower
            from tamp.ros_utils import goal_to_ros, ros_to_task_plan

            print("Waiting for planning server...")
            rospy.wait_for_service("get_latest_plan")
            self.goal_to_ros = goal_to_ros
            self.ros_to_task_plan = ros_to_task_plan
            self.init_state_client = rospy.ServiceProxy(
                "/reset_planning", SetPlanningState)
            self.get_plan_client = rospy.ServiceProxy(
                "/get_latest_plan", GetPlan)
            if self.use_learning_server:
                self.learning_server = rospy.Service(
                    "/plan_tower", PlanTower, self.plan_and_execute_tower)
                print("Learning server started!")
            print("Done!")
        elif self.use_learning_server:
            print('Starting server...')
            from stacking_ros.srv import PlanGrasp, GetPose

            self.pose_server = rospy.Service(
                "/get_pose",
                GetPose,
                self.plan_and_execute_look
            )
            self.learning_server = rospy.Service(
                "/plan_grasp",
                PlanGrasp,
                self.plan_and_execute_grasp
            )
            print("Learning server started!")
        else:
            # Looks like this variable is never used anymore/broken.
            self.planning_client = None

        # TODO: (ICRA) Update PDDL Stream Domain (planning probably isn't needed).
        # self.pddl_info = get_pddlstream_info(
        #     self.robot,
        #     self.fixed,
        #     self.pddl_blocks,
        #     add_slanted_grasps=False,
        #     approach_frame='global',
        #     use_vision=self.use_vision
        # )

        self.noise = noise
        self.teleport = teleport
        self.txt_id = None
        self.plan()
    
    def execute(self):
        self.state = 'execute'
        pb_robot.aabb.set_client(self._execution_client_id)
        pb_robot.body.set_client(self._execution_client_id)
        pb_robot.collisions.set_client(self._execution_client_id)
        pb_robot.geometry.set_client(self._execution_client_id)
        pb_robot.grasp.set_client(self._execution_client_id)
        pb_robot.joint.set_client(self._execution_client_id)
        pb_robot.link.set_client(self._execution_client_id)
        pb_robot.panda.set_client(self._execution_client_id)
        pb_robot.planning.set_client(self._execution_client_id)
        pb_robot.utils.set_client(self._execution_client_id)
        pb_robot.viz.set_client(self._execution_client_id)

    def plan(self):
        if self.use_action_server:
            return
        self.state = 'plan'
        pb_robot.aabb.set_client(self._planning_client_id)
        pb_robot.body.set_client(self._planning_client_id)
        pb_robot.collisions.set_client(self._planning_client_id)
        pb_robot.geometry.set_client(self._planning_client_id)
        pb_robot.grasp.set_client(self._planning_client_id)
        pb_robot.joint.set_client(self._planning_client_id)
        pb_robot.link.set_client(self._planning_client_id)
        pb_robot.panda.set_client(self._planning_client_id)
        pb_robot.planning.set_client(self._planning_client_id)
        pb_robot.utils.set_client(self._planning_client_id)
        pb_robot.viz.set_client(self._planning_client_id)
    
    def reset_world(self):
        """ Resets the planning world to its original configuration """
        print("Resetting world")
        self.plan()
        self.robot.arm.SetJointValues(self.orig_joint_angles)
        self.execute()
        self.execution_robot.arm.SetJointValues(self.orig_joint_angles)
        for bx, b in enumerate(self.pddl_blocks):
            b.set_base_link_pose(self.orig_block_poses[bx])
        print("Done")


def sample_platform_poses(bd_body, n=3):
    x1 = np.random.uniform(0.3, 0.4)
    y1 = np.random.uniform(0.8, 0.9)


def sample_block_poses(bd_body, n=3):

    options = ((1,1), (1,0), (1,2), (0,2), (2,2), (2,0), (2,1), (0,1))

    # dx = np.random.uniform(-0.00, 0.00)
    # dy = np.random.uniform(-0.00, 0.00)

    # x1 = get_pose(bd_body['region1'])[0][0] + dx
    # y1 = get_pose(bd_body['region1'])[0][1] + dy

    # x_zero = 0.0
    # dx_minus = np.random.uniform(-0.07, -0.055)
    # dx_plus = np.random.uniform(0.055, 0.07)

    # y_zero = 0.0
    # dy_minus = np.random.uniform(-0.07, -0.055)
    # dy_plus = np.random.uniform(0.055, 0.07)

    # id = np.random.choice(len(options), 2, replace=False)
    # ids = [options[id[0]], options[id[1]]]

    dx = np.random.uniform(-0.03, 0.03)
    dy = np.random.uniform(-0.03, 0.03)

    x1 = get_pose(bd_body['region1'])[0][0] + dx
    y1 = get_pose(bd_body['region1'])[0][1] + dy

    x_zero = 0.0
    dx_minus = np.random.uniform(-0.055, -0.055)
    dx_plus = np.random.uniform(0.055, 0.055)

    y_zero = 0.0
    dy_minus = np.random.uniform(-0.055, -0.055)
    dy_plus = np.random.uniform(0.055, 0.055)

    id1 = np.random.choice(8)
    id2 = id1 + np.random.choice((-1,1))
    if id2 > 7:
        id2 = 0
    ids = [options[id1], options[id2]]

    xs = [x_zero, dx_minus, dx_plus]
    ys = [y_zero, dy_minus, dy_plus]


    x2 = x1 + xs[ids[0][0]]
    y2 = y1 + ys[ids[0][1]]
    x3 = x1 + xs[ids[1][0]]
    y3 = y1 + ys[ids[1][1]]

    assert (x2, y2) != (x3, y3) != (x1, y1)

    c1_pose = None
    c2_pose = None
    c3_pose = None

    poses = {}
    # c1_pose = Pose(Point(x=x1, y=y1, z=stable_z(bd_body['c1'], bd_body['region1'])))
    poses['c1'] = (x1, y1)
    if n > 1:
        # c2_pose = Pose(Point(x=x2, y=y2, z=stable_z(bd_body['c2'], bd_body['region1'])))
        # poses['c2'] = c2_pose
        poses['c2'] = (x2, y2)
    if n > 2:
        # c3_pose = Pose(Point(x=x3, y=y3, z=stable_z(bd_body['c3'], bd_body['region1'])))
        # poses['c3'] = c3_pose
        poses['c3'] = (x3, y3)

    return poses

#######################################################

# def setup_panda_world(robot, blocks, xy_poses=None, use_platform=True):
#     # Adjust robot position such that measurements match real robot reference frame
#     import os
#     import pb_robot
#     import shutil
#     from tamp.misc import create_pb_robot_urdf
#     from block_utils import Object, Pose, Position, all_rotations

#     robot_pose = np.eye(4)
#     robot.set_transform(robot_pose)

#     pddl_blocks = []

#     full_urdf_folder = 'pb_robot/tmp_urdfs'

#     if not os.path.exists(full_urdf_folder):
#         os.makedirs(full_urdf_folder)

#     for block in blocks:
#         pb_block_fname = create_pb_robot_urdf(block, block.name + '.urdf')
#         pddl_block = pb_robot.body.createBody(pb_block_fname)
#         pddl_blocks.append(pddl_block)

#     table_x_offset = 0.2
#     floor_path = 'tamp/models/panda_table.urdf'
#     shutil.copyfile(floor_path, 'pb_robot/models/panda_table.urdf')
#     table_file = os.path.join('models', 'panda_table.urdf')
#     pddl_table = pb_robot.body.createBody(table_file)
#     pddl_table.set_point([table_x_offset, 0, 0])

#     frame_path = 'tamp/models/panda_frame.urdf'
#     shutil.copyfile(frame_path, 'pb_robot/models/panda_frame.urdf')
#     frame_file = os.path.join('models', 'panda_frame.urdf')
#     pddl_frame = pb_robot.body.createBody(frame_file)
#     pddl_frame.set_point([table_x_offset + 0.762 - 0.0127, 0 + 0.6096 - 0.0127, 0])

#     wall_path = 'tamp/models/walls.urdf'
#     shutil.copyfile(wall_path, 'pb_robot/models/walls.urdf')
#     wall_file = os.path.join('models', 'walls.urdf')
#     pddl_wall = pb_robot.body.createBody(wall_file)
#     pddl_wall.set_point([table_x_offset + 0.762 + 0.005, 0, 0])

#     # Set the initial positions randomly on table.
#     if xy_poses is None:
#         storage_poses = [(-0.4, -0.45), (-0.4, -0.25), # Left Corner
#                          (-0.25, -0.5), (-0.4, 0.25),   # Back Center
#                          (-0.4, 0.45), (-0.25, 0.5),   # Right Corner
#                          (-0., -0.5), (0., -0.35),   # Left Side
#                          (-0., 0.5), (0., 0.35)]     # Right Side
#         print('Placing blocks in storage locations...')
#         for ix, block in enumerate(pddl_blocks):
#             x, y = storage_poses[ix]
#             dimensions = np.array(block.get_dimensions()).reshape((3, 1))
#             if ix < 6 and (ix not in [2, 5]):  # Back storage should have long side along y-axis.
#                 for rot in all_rotations():
#                     rot_dims = np.abs(rot.as_matrix()@dimensions)[:, 0]
#                     if rot_dims[1] >= rot_dims[0] and rot_dims[1] >= rot_dims[2]:
#                         block.set_base_link_pose(((x, y, 0.), rot.as_quat()))
#                         break
#             else:  # Side storage should have long side along x-axis.
#                 for rot in all_rotations():
#                     rot_dims = np.abs(rot.as_matrix()@dimensions)[:, 0]
#                     if rot_dims[0] >= rot_dims[1] and rot_dims[0] >= rot_dims[2]:
#                         block.set_base_link_pose(((x, y, 0.), rot.as_quat()))
#                         break

#             z = pb_robot.placements.stable_z(block, pddl_table)
#             block.set_base_link_point([x, y, z])
#     else:
#         for i, (block, xy_pose) in enumerate(zip(pddl_blocks, xy_poses)):

#             full_pose = Pose(Position(xy_pose.pos.x,
#                                      xy_pose.pos.y,
#                                      xy_pose.pos.z),
#                             xy_pose.orn)
#             block.set_base_link_pose(full_pose)
#             z = pb_robot.placements.stable_z(block, pddl_table)
#             block.set_base_link_point([xy_pose.pos.x, xy_pose.pos.y, z])


#     # Setup platform.
#     if use_platform:
#         platform, leg = Object.platform()
#         pb_platform_fname = create_pb_robot_urdf(platform, 'platform.urdf')
#         pb_leg_fname = create_pb_robot_urdf(leg, 'leg.urdf')
#         pddl_platform = pb_robot.body.createBody(pb_platform_fname)
#         pddl_leg = pb_robot.body.createBody(pb_leg_fname)

#         rotation = pb_robot.geometry.Euler(yaw=np.pi/2)
#         pddl_platform.set_base_link_pose(pb_robot.geometry.multiply(pb_robot.geometry.Pose(euler=rotation), pddl_platform.get_base_link_pose()))
#         pddl_leg.set_base_link_pose(pb_robot.geometry.multiply(pb_robot.geometry.Pose(euler=rotation), pddl_leg.get_base_link_pose()))

#         table_z = pddl_table.get_base_link_pose()[0][2]
#         pddl_leg.set_base_link_point([0.7, -0.4, table_z + leg.dimensions.z/2])
#         pddl_platform.set_base_link_point([0.7, -0.4, table_z + leg.dimensions.z + platform.dimensions.z/2.])
#     else:
#         pddl_platform = None
#         pddl_leg = None

#     return pddl_blocks, pddl_platform, pddl_leg, pddl_table, pddl_frame, pddl_wall


#######################################################



def display_scenario():
    connect(use_gui=True)

    scn = Scene_unpack1()
    scn.reset()

    for i in range(10000):
        step_simulation()
        time.sleep(0.1)

    disconnect()
    print('Finished.')


if __name__ == '__main__':
    display_scenario()
