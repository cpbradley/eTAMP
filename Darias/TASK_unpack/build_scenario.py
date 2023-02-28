#!/usr/bin/env python

from __future__ import print_function
import numpy as np

import time
from utils.pybullet_tools.kuka_primitives3 import BodyPose, BodyConf, Register
from utils.pybullet_tools.utils import WorldSaver, connect, dump_world, get_pose, set_pose, Pose, \
    Point, set_default_camera, stable_z, disconnect, get_bodies, HideOutput, \
    create_box, \
    load_pybullet, step_simulation, Euler, get_links, get_link_info, get_movable_joints, set_joint_positions, \
    set_camera, get_center_extent, tform_from_pose, attach_viewcone, LockRenderer, get_aabb

from utils.pybullet_tools.body_utils import draw_frame

from copy import copy


class Scene_unpack1(object):
    def __init__(self):
        with HideOutput():
            with LockRenderer():
                self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
                                              fixed_base=True)
                self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

                self.bd_body = {
                    'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
                    'cabinet_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
                        fixed_base=True),
                    'drawer_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
                        fixed_base=True),
                    'pegboard': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
                        fixed_base=True),
                    # 'region1': load_pybullet("../scenario_description/region.urdf", fixed_base=True),
                    'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
                    'region2': load_pybullet("../scenario_description/region_big.urdf",
                                             fixed_base=True),
                    'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
                }
                self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

                self.drawer_links = get_links(self.bd_body['drawer_shelf'])
                cabinet_links = get_links(self.bd_body['cabinet_shelf'])

                set_pose(self.bd_body['cabinet_shelf'],
                         Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['drawer_shelf'],
                         Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['pegboard'],
                         Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
                set_pose(self.bd_body['region1'],
                        #  Pose(Point(x=0.35, y=0.9, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                         Pose(Point(x=0.45, y=0.7, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                set_pose(self.bd_body['region2'],
                         Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

                self.movable_bodies = [self.bd_body['c1'], ]
                self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
                                   self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
                self.regions = [self.bd_body['region1'], self.bd_body['region2']]

                self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

                self.sensors = []

                self.robots = [self.arm_left]

                self.dic_body_info = {}
                for b in self.movable_bodies:
                    obj_center, obj_extent = get_center_extent(b)
                    body_pose = get_pose(b)
                    body_frame = tform_from_pose(body_pose)
                    bottom_center = copy(obj_center)
                    bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
                    bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
                    relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
                    center_frame = tform_from_pose((obj_center, body_pose[1]))
                    relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

                    self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

                self.init_poses = sample_block_poses(self.bd_body, n=1)
                self.reset()

    def reset(self):
        with HideOutput():
            with LockRenderer():
                # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
                initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
                config_left = BodyConf(self.arm_left, initial_jts)
                config_left.assign()

                movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
                set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

                # set_pose(self.bd_body['c1'],
                #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
                for key, value in self.init_poses.items():
                    set_pose(self.bd_body[key],
                            Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))


                set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))

    def get_elemetns(self):
        self.reset()
        return self.arm_left, self.movable_bodies, self.regions

class Scene_unpack2(object):
    def __init__(self):
        with HideOutput():
            with LockRenderer():
                self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
                                              fixed_base=True)
                self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

                self.bd_body = {
                    'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
                    'cabinet_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
                        fixed_base=True),
                    'drawer_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
                        fixed_base=True),
                    'pegboard': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
                        fixed_base=True),
                    # 'region1': load_pybullet("../scenario_description/region.urdf", fixed_base=True),
                    'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
                    'region2': load_pybullet("../scenario_description/region_big.urdf",
                                             fixed_base=True),
                    'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
                    'c2': load_pybullet("../scenario_description/boxC.urdf", fixed_base=False),
                }
                self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

                self.drawer_links = get_links(self.bd_body['drawer_shelf'])
                cabinet_links = get_links(self.bd_body['cabinet_shelf'])

                set_pose(self.bd_body['cabinet_shelf'],
                         Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['drawer_shelf'],
                         Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['pegboard'],
                         Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
                set_pose(self.bd_body['region1'],
                        #  Pose(Point(x=0.35, y=0.9, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                         Pose(Point(x=0.45, y=0.7, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                set_pose(self.bd_body['region2'],
                         Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

                self.movable_bodies = [self.bd_body['c1'], self.bd_body['c2'], ]
                self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
                                   self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
                self.regions = [self.bd_body['region1'], self.bd_body['region2']]

                self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

                self.sensors = []

                self.robots = [self.arm_left]

                self.dic_body_info = {}
                for b in self.movable_bodies:
                    obj_center, obj_extent = get_center_extent(b)
                    body_pose = get_pose(b)
                    body_frame = tform_from_pose(body_pose)
                    bottom_center = copy(obj_center)
                    bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
                    bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
                    relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
                    center_frame = tform_from_pose((obj_center, body_pose[1]))
                    relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

                    self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

                self.init_poses = sample_block_poses(self.bd_body, n=2)
                self.reset()

    def reset(self):
        with HideOutput():
            with LockRenderer():
                # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
                initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
                config_left = BodyConf(self.arm_left, initial_jts)
                config_left.assign()

                movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
                set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

                # set_pose(self.bd_body['c1'],
                #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
                # set_pose(self.bd_body['c2'],
                #          Pose(Point(x=0.32, y=0.9, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
                for key, value in self.init_poses.items():
                    set_pose(self.bd_body[key],
                            Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))


                set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))

    def get_elemetns(self):
        self.reset()
        return self.arm_left, self.movable_bodies, self.regions


class Scene_unpack3(object):
    def __init__(self):
        with HideOutput():
            with LockRenderer():
                self.arm_left = load_pybullet("../darias_description/urdf/darias_L_primitive_collision.urdf",
                                              fixed_base=True)
                self.arm_base = load_pybullet("../darias_description/urdf/darias_base.urdf", fixed_base=True)

                self.bd_body = {
                    'floor': load_pybullet("../scenario_description/floor.urdf", fixed_base=True),
                    'cabinet_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/cabinet_shelf.urdf",
                        fixed_base=True),
                    'drawer_shelf': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/drawer_shelf.urdf",
                        fixed_base=True),
                    'pegboard': load_pybullet(
                        "../scenario_description/manipulation_worlds/urdf/pegboard.urdf",
                        fixed_base=True),
                    'region1': load_pybullet("../scenario_description/region_big.urdf", fixed_base=True),
                    'region2': load_pybullet("../scenario_description/region_big.urdf",
                                             fixed_base=True),
                    'c1': load_pybullet("../scenario_description/boxCm.urdf", fixed_base=False),
                    'c2': load_pybullet("../scenario_description/boxC.urdf", fixed_base=False),
                    'c3': load_pybullet("../scenario_description/boxCx.urdf", fixed_base=False),
                }
                self.bd_body.update(dict((self.bd_body[k], k) for k in self.bd_body))

                self.drawer_links = get_links(self.bd_body['drawer_shelf'])
                cabinet_links = get_links(self.bd_body['cabinet_shelf'])

                set_pose(self.bd_body['cabinet_shelf'],
                         Pose(Point(x=-0.45, y=-0.8, z=stable_z(self.bd_body['cabinet_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['drawer_shelf'],
                         Pose(Point(x=-0.45, y=0.8, z=stable_z(self.bd_body['drawer_shelf'], self.bd_body['floor']))))
                set_pose(self.bd_body['pegboard'],
                         Pose(Point(x=-0.60, y=0, z=stable_z(self.bd_body['pegboard'], self.bd_body['floor']))))
                set_pose(self.bd_body['region1'],
                         Pose(Point(x=0.45, y=0.8, z=stable_z(self.bd_body['region1'], self.bd_body['floor']))))
                set_pose(self.bd_body['region2'],
                         Pose(Point(x=0.05, y=0.8, z=stable_z(self.bd_body['region2'], self.bd_body['floor']))))

                self.movable_bodies = [self.bd_body['c1'], self.bd_body['c2'], self.bd_body['c3']]
                self.env_bodies = [self.arm_base, self.bd_body['floor'], self.bd_body['cabinet_shelf'],
                                   self.bd_body['drawer_shelf'], self.bd_body['pegboard']]
                self.regions = [self.bd_body['region1'], self.bd_body['region2']]

                self.all_bodies = list(set(self.movable_bodies) | set(self.env_bodies) | set(self.regions))

                self.sensors = []

                self.robots = [self.arm_left]

                self.dic_body_info = {}
                for b in self.movable_bodies:
                    obj_center, obj_extent = get_center_extent(b)
                    body_pose = get_pose(b)
                    body_frame = tform_from_pose(body_pose)
                    bottom_center = copy(obj_center)
                    bottom_center[2] = bottom_center[2] - obj_extent[2] / 2
                    bottom_frame = tform_from_pose((bottom_center, body_pose[1]))
                    relative_frame_bottom = np.dot(bottom_frame, np.linalg.inv(body_frame))  # from pose to bottom
                    center_frame = tform_from_pose((obj_center, body_pose[1]))
                    relative_frame_center = np.dot(center_frame, np.linalg.inv(body_frame))

                    self.dic_body_info[b] = (obj_extent, relative_frame_bottom, relative_frame_center)

                self.init_poses = sample_block_poses(self.bd_body)
                # print(self.init_poses)
                # assert False
                self.reset()

    def reset(self):
        with HideOutput():
            with LockRenderer():
                # initial_jts = np.array([0.8, 0.75, 0.4, -1.8, 0.8, -1.5, 0])
                initial_jts = np.array([0.1, 1.4, 1, 1.7, 0, 0, 0])
                config_left = BodyConf(self.arm_left, initial_jts)
                config_left.assign()

                movable_door = get_movable_joints(self.bd_body['cabinet_shelf'])
                set_joint_positions(self.bd_body['cabinet_shelf'], movable_door, [-0.])

                # set_pose(self.bd_body['c1'],
                #          Pose(Point(x=0.375, y=0.9, z=stable_z(self.bd_body['c1'], self.bd_body['region1']))))
                # set_pose(self.bd_body['c2'],
                #          Pose(Point(x=0.32, y=0.9, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
                #         #  Pose(Point(x=0.02, y=0.7, z=stable_z(self.bd_body['c2'], self.bd_body['region1']))))
                # set_pose(self.bd_body['c3'],
                #          Pose(Point(x=0.07, y=0.845, z=stable_z(self.bd_body['c3'], self.bd_body['region1']))))

                for key, value in self.init_poses.items():
                    set_pose(self.bd_body[key],
                            Pose(Point(x=value[0], y=value[1], z=stable_z(self.bd_body[key], self.bd_body['region1']))))

                set_camera(150, -35, 1.6, Point(-0.1, 0.1, -0.1))
    
        
    def get_elemetns(self):
        self.reset()
        return self.arm_left, self.movable_bodies, self.regions


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
