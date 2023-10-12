import numpy as np
import random
import pybullet as p

import pb_robot
from pb_robot.tsrs.panda_box import ComputePrePose

from block_utils import rotation_group, ZERO_POS, all_rotations
from pybullet_utils import transformation

from scipy.spatial.transform import Rotation as R


rotations = all_rotations()
DEBUG_FAILURE = False

class sdg_fn(object):
    def __init__(self, gen_fn, *args, **kwargs):
        self.args = args
        print(args)
        print(kwargs)
        if args:
            if kwargs:
                self.fn = gen_fn(*args, **kwargs)
            else:
                self.fn = gen_fn(*args)
        elif kwargs:
            self.fn = gen_fn(**kwargs)
        else:
            self.fn = gen_fn()

        # self.dic_body_info = dic_body_info
        # self.robot = robot
        # self.end_effector_link = link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])

    def __call__(self, input_tuple, fluent_tuple=(), seed=None):
        if fluent_tuple:
            return self.fn(*input_tuple, fluents=fluent_tuple, seed=seed)
        else:
            return self.fn(*input_tuple, seed=seed)

# class sdg_sample_grasp(object):
#     def __init__(self, robot, dic_body_info=None):
#         self.dic_body_info = dic_body_info
#         self.robot = robot
#         self.end_effector_link = link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])

#     def search(self, input_tuple, seed=None):
#         """return the ee_frame wrt the object_frame of the object"""
#         try:
#             body, pose, grasp_dir = input_tuple  # grasp_dir defined in ellipsoid_frame of the body
#         except:
#             body, pose = input_tuple  # grasp_dir defined in ellipsoid_frame of the body
#             direction = 2
#             grasp_dir = GraspDirection(body, pose, direction, self.robot, self.dic_body_info)

#         assert body == grasp_dir.body
#         ellipsoid_frame, obj_extent, direction = grasp_dir.ellipsoid_frame, grasp_dir.obj_extent, grasp_dir.direction

#         ex, ey, ez = obj_extent

#         translate_z = Pose(point=[0, 0, -0.001])
#         list_grasp = []
#         list_ind = [0, 1]
#         if direction == 0:
#             """ee at +X of the ellipsoid_frame"""
#             swap_z = Pose(euler=[0, -np.pi / 2, 0])  # direct +z to -x
#             # translate_point: choose from the grasping surface with 2 dof
#             d1, d2 = 0., 0.  # [-0.5, 0.5]
#             translate_point = Pose(point=[ex / 2, 0 + d1 * ey, ez / 2 + d2 * ez])
#             for j in range(2):
#                 rotate_z = Pose(euler=[0, 0, j * np.pi])  # gripper open with +Y direction
#                 grasp = multiply(translate_point, swap_z, rotate_z, translate_z)
#                 list_grasp.append(grasp)

#         elif direction == 1:
#             """ee at +Y"""
#             swap_z = Pose(euler=[np.pi / 2, 0, 0])  # direct +z to -y
#             d1, d2 = 0., 0.  # [-0.5, 0.5]
#             translate_point = Pose(point=[0 - d1 * ex, ey / 2, ez / 2 + d2 * ez])
#             for j in range(2):
#                 rotate_z = Pose(euler=[0, 0, j * np.pi + np.pi / 2])
#                 grasp = multiply(translate_point, swap_z, rotate_z, translate_z)
#                 list_grasp.append(grasp)

#         elif direction == 2:
#             """ee at +Z"""
#             list_ind = [0, 1, 2, 3]
#             swap_z = Pose(euler=[0, np.pi, 0])  # direct +z to -z
#             d1, d2 = 0., 0.  # [-0.5, 0.5]
#             translate_point = Pose(point=[0 - d2 * ex, 0 + d1 * ey, ez])
#             for j in range(4):
#                 rotate_z = Pose(euler=[0, 0, j * np.pi / 2])
#                 grasp = multiply(translate_point, swap_z, rotate_z, translate_z)
#                 list_grasp.append(grasp)

#         elif direction == 3:
#             """ee at -X"""
#             swap_z = Pose(euler=[0, np.pi / 2, 0])
#             d1, d2 = 0., 0.  # [-0.5, 0.5]
#             translate_point = Pose(point=[-ex / 2, 0 - d1 * ey, ez / 2 + d2 * ez])
#             for j in range(2):
#                 rotate_z = Pose(euler=[0, 0, j * np.pi + np.pi])
#                 grasp = multiply(translate_point, swap_z, rotate_z, translate_z)
#                 list_grasp.append(grasp)

#         elif direction == 4:
#             """ee at -Y"""
#             swap_z = Pose(euler=[-np.pi / 2, 0, 0])
#             d1, d2 = 0., 0.  # [-0.5, 0.5]
#             translate_point = Pose(point=[0 + d1 * ex, -ey / 2, ez / 2 + d2 * ez])
#             for j in range(2):
#                 rotate_z = Pose(euler=[0, 0, j * np.pi - np.pi / 2])
#                 grasp = multiply(translate_point, swap_z, rotate_z, translate_z)
#                 list_grasp.append(grasp)

#         """ee_frame wrt ellipsoid_frame"""
#         # if seed is None:
#         #     grasp_pose = random.sample(list_grasp, 1)[0]
#         # else:
#         if seed is None:
#             # assert False
#             idx = random.sample(list_ind, 1)[0]
#         else:
#             idx = np.array([seed]).flatten()[0]
#             if idx > len(list_grasp)-1:
#                 list_grasp = list_grasp*2
#         # print(f'SEEEEEEEEEEEEEEED: {seed}')
#         # print(idx)
#         # print(list_grasp)
#         if idx > len(list_grasp):
#             assert False

#         grasp_pose = list_grasp[int(idx)]
#         # print(grasp_pose)
#         """ee_frame wrt object_frame: get_pose()"""
#         grasp_pose = multiply(invert(get_pose(body)), pose_from_tform(ellipsoid_frame), grasp_pose)

#         approach_pose = Pose(0.1 * Point(z=-1))  # pose bias wrt end-effector frame
#         body_grasp = BodyGrasp(body, grasp_pose, approach_pose, self.robot, self.end_effector_link)
#         return (body_grasp,)  # return a tuple

#     def __call__(self, input_tuple, seed=None):
#         return self.search(input_tuple, seed=seed)

def get_grasp_gen(robot, add_slanted_grasps=False, add_orthogonal_grasps=True):
    # add_slanted_grasps = True
    # I opt to use TSR to define grasp sets but you could replace this
    # with your favorite grasp generator
    def gen(body, pose, seed=None):
        # Note, add_slanted_grasps should be True when we're using the platform.
        # body, pose = input
        print(seed)
        grasp_tsr = pb_robot.tsrs.panda_box.grasp(body,
            add_slanted_grasps=add_slanted_grasps, add_orthogonal_grasps=add_orthogonal_grasps)
        grasps = []

        # np.random.shuffle(grasp_tsr)
        top_grasps = []
        for sampled_tsr in grasp_tsr:
            grasp_worldF = sampled_tsr.sample()
            grasp_objF = np.dot(np.linalg.inv(body.get_base_link_transform()), grasp_worldF)
            grasp_worldR = grasp_worldF[:3,:3]
            e_x, e_y, e_z = np.eye(3) # basis vectors
            is_top_grasp = grasp_worldR[:,2].dot(-e_z) > 0.999
            is_upside_down_grasp = grasp_worldR[:,2].dot(e_z) > 0.001
            if is_top_grasp:
                assert not is_upside_down_grasp
            
            body_grasp = pb_robot.vobj.BodyGrasp(body, grasp_objF, robot.arm)
            grasps.append((body_grasp,))
            # yield (body_grasp,)
            if is_top_grasp:
                top_grasps.append(body_grasp)
                # return (body_grasp,)
        if seed is None:
            assert False
            list_ind = [0, 1, 2, 3]
            idx = random.sample(list_ind, 1)[0]
        else:
            idx = np.array([seed]).flatten()[0]
            if idx > len(top_grasps)-1:
                top_grasps = top_grasps*2
        if idx > len(top_grasps):
            assert False
        print(top_grasps)
        print(idx)
        print(grasp_tsr)
        print(body)
        grasp = (top_grasps[int(idx)],)
        print(grasp)
        # assert False
        return grasp

    # def gen(body):
    #     dims = body.get_dimensions()


    return gen


def get_stable_gen_table(fixed=[]):
    def gen(body, surface, surface_pos=None, protation=None, seed=None):
        """
        Generate a random pose (possibly rotated) on a surface. Rotation
        can be specified for debugging.
        """
        # These poses are useful for regrasping. Poses are more useful for grasping
        # if they are upright.
        dims = body.get_dimensions()

        poses = []
        surface_position = surface.get_pose()[0]
        while True:
            # These are the pre-chosen regrap locations.
            # x = surface_position[0] + np.random.uniform(0.1, 0.3)
            # y = np.random.uniform(-0.2, 0.2)
            x = surface_position[0] + 0.2 + random.choice((-0.15, 0.15))
            y = surface_position[1] + random.choice((-0.15, 0.15))
            # rotation = random.choice(rotations)
            yaw = np.random.uniform(-np.pi, np.pi)
            # rotation = R.from_euler('zyx', [0, 0, 0.])
            rotation = R.from_euler('zyx', [0, 0, 0.])
            rotation = body.get_base_link_pose()[1]

            start_pose = body.get_base_link_pose()
            orig_z = start_pose[0][2]

            # pose = (ZERO_POS, rotation.as_quat())
            pose = (ZERO_POS, rotation)
            body.set_base_link_pose(pose)
            z = pb_robot.placements.stable_z(body, surface)
            # print(z)
            # print(orig_z)
            # if z != orig_z:
            #     if DEBUG_FAILURE: input('Bad z')
            # pose = ((x, y, z), rotation.as_quat())
            pose = ((x, y, z), rotation)

            # Check if regrasp pose is valid.
            body.set_base_link_pose(pose)
            if (pose is None) or any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
                body.set_base_link_pose(start_pose)
                'Sampling ...'
                continue
            body.set_base_link_pose(start_pose)

            body_pose = pb_robot.vobj.BodyPose(body, pose)
            return (body_pose,)
        # poses.append((body_pose,))

        # for x, y in [(0.4, 0.4)]:
        #     np.random.shuffle(rotations)
        #     for rotation in rotations:
        #         start_pose = body.get_base_link_pose()

        #         # Get regrasp pose.
        #         pose = (ZERO_POS, rotation.as_quat())
        #         body.set_base_link_pose(pose)
        #         z = pb_robot.placements.stable_z(body, surface)
        #         pose = ((x, y, z), rotation.as_quat())

        #         # Check if regrasp pose is valid.
        #         body.set_base_link_pose(pose)
        #         if (pose is None) or any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
        #             body.set_base_link_pose(start_pose)
        #             continue
        #         body.set_base_link_pose(start_pose)

        #         body_pose = pb_robot.vobj.BodyPose(body, pose)
        #         poses.append((body_pose,))
        # return poses
    return gen


def get_stable_gen_home(home_poses, fixed=[]):
    def gen(body, surface, surface_pos, protation=None, seed=None):
        """
        Generate a random pose (possibly rotated) on a surface. Rotation
        can be specified for debugging.
        """
        # These poses are useful for regrasping. Poses are more useful for grasping
        # if they are upright.
        dims = body.get_dimensions()

        poses = []
        home_pose = home_poses[body.get_name()]
        np.random.shuffle(rotations)
        for rotation in rotations:
            start_pose = body.get_base_link_pose()

            # Get regrasp pose.
            pose = (ZERO_POS, rotation.as_quat())
            body.set_base_link_pose(pose)
            z = pb_robot.placements.stable_z(body, surface)
            x, y = home_pose[0][0:2]
            pose = ((x, y, z), rotation.as_quat())

            # Check if regrasp pose is valid.
            body.set_base_link_pose(pose)
            if (pose is None) or any(pb_robot.collisions.pairwise_collision(body, b) for b in fixed):
                body.set_base_link_pose(start_pose)
                continue
            body.set_base_link_pose(start_pose)

            body_pose = pb_robot.vobj.BodyPose(body, pose)
            poses.append((body_pose,))
        return poses
    return gen



def get_stable_gen_block(fixed=[]):
    def fn(body, surface, surface_pose, rel_pose, seed=None):
        """
        @param rel_pose: A homogeneous transformation matrix.
        """
        surface_tform = pb_robot.geometry.tform_from_pose(surface_pose.pose)
        body_tform = surface_tform@rel_pose
        pose = pb_robot.geometry.pose_from_tform(body_tform)
        body_pose = pb_robot.vobj.BodyPose(body, pose)
        return (body_pose,)
    return fn


def get_ik_fn(robot, fixed=[], num_attempts=10, approach_frame='gripper', backoff_frame='global', use_wrist_camera=False):
    def fn(body, pose, grasp, fluents=[], return_grasp_q=False, check_robust=False, seed=None):
        obstacles = assign_fluent_state(fluents)
        fluent_names = [o.get_name() for o in obstacles]
        for o in fixed:
            if o.get_name() not in fluent_names:
                obstacles.append(o)
        
        obstacles += [body]

        obj_worldF = pb_robot.geometry.tform_from_pose(pose.pose)
        grasp_worldF = np.dot(obj_worldF, grasp.grasp_objF)
        grasp_worldR = grasp_worldF[:3,:3]

        e_x, e_y, e_z = np.eye(3) # basis vectors

        # The x-axis of the gripper points toward the camera
        # The y-axis of the gripper points along the plane of the hand
        # The z-axis of the gripper points forward

        is_top_grasp = grasp_worldR[:,2].dot(-e_z) > 0.999
        is_upside_down_grasp = grasp_worldR[:,2].dot(e_z) > 0.001
        is_gripper_sideways = np.abs(grasp_worldR[:,1].dot(e_z)) > 0.999
        is_camera_down = grasp_worldR[:,0].dot(-e_z) > 0.999
        is_wrist_too_low = grasp_worldF[2,3] < 0.088/2 + 0.005

        print('Attempting IK')


        if is_gripper_sideways:
            print('Sideways')
            return None
        if is_upside_down_grasp:
            print('Upside Down')
            return None
        if is_camera_down:# and approach_frame == 'gripper':
            print('Camera?')
            return None

        # the gripper is too close to the ground. the wrist of the arm is 88mm
        # in diameter, and it is the widest part of the hand. Include a 5mm
        # clearance
        if not is_top_grasp and is_wrist_too_low:
            print('Not top')
            return None
        # If the block/gripper is in the storage area, don't use low grasps.
        if grasp_worldF[0,3] < 0.2 and grasp_worldF[2,3] < 0.1:
            print('Storage')
            return None

        if approach_frame == 'gripper':
            approach_tform = ComputePrePose(grasp_worldF, [0, 0, -0.1], approach_frame)
        elif approach_frame == 'global':
            approach_tform = ComputePrePose(grasp_worldF, [0, 0, 0.1], approach_frame) # Was -0.125
        else:
            raise NotImplementedError()

        if backoff_frame == 'gripper':
            backoff_tform = ComputePrePose(grasp_worldF, [0, 0, -0.1], backoff_frame)
        elif backoff_frame == 'global':
            backoff_tform = ComputePrePose(grasp_worldF, [0, 0, 0.1], backoff_frame) # Was -0.125
        else:
            raise NotImplementedError()

        for ax in range(num_attempts):
            print('Attempt', ax)
            print(pose.pose)
            q_grasp = robot.arm.ComputeIK(grasp_worldF)
            if (q_grasp is None):
                if DEBUG_FAILURE: input('No Grasp IK')
                continue
            if not robot.arm.IsCollisionFree(q_grasp, self_collisions=False, obstacles=obstacles, debug=DEBUG_FAILURE):
                if DEBUG_FAILURE: input('Grasp collision')
                continue
            else:
                print('Grasp IK found')
                print(obstacles)

            q_approach = robot.arm.ComputeIK(approach_tform, seed_q=q_grasp)
            if (q_approach is None):
                if DEBUG_FAILURE: input('No approach IK')
                continue
            if not robot.arm.IsCollisionFree(q_approach, obstacles=obstacles, debug=DEBUG_FAILURE):
                if DEBUG_FAILURE: input('Approach motion collision')
                continue
            conf_approach = pb_robot.vobj.BodyConf(robot, q_approach)


            # Only recompute the backoff if it's different from the approach.
            if approach_frame == backoff_frame:
                q_backoff = q_approach
            else:
                q_backoff = robot.arm.ComputeIK(backoff_tform, seed_q=q_grasp)
                if (q_backoff is None):
                    if DEBUG_FAILURE: input('No backoff IK')
                    continue
                if not robot.arm.IsCollisionFree(q_backoff, obstacles=obstacles, debug=DEBUG_FAILURE):
                    if DEBUG_FAILURE: input('Backoff motion collision')
                    continue
            conf_backoff = pb_robot.vobj.BodyConf(robot, q_backoff)

            path_approach = robot.arm.snap.PlanToConfiguration(robot.arm, q_approach, q_grasp, obstacles=obstacles)
            if path_approach is None:
                if DEBUG_FAILURE: input('Approach motion failed')
                continue
            if backoff_frame == 'global':
                path_backoff = robot.arm.snap.PlanToConfiguration(robot.arm, q_grasp, q_backoff, obstacles=obstacles, check_upwards=True)
            else:
                path_backoff = robot.arm.snap.PlanToConfiguration(robot.arm, q_grasp, q_backoff, obstacles=obstacles, check_upwards=False)
            if path_backoff is None:
                if DEBUG_FAILURE: input('Backoff motion failed')
                continue

            # If the grasp is valid, check that it is robust (i.e., also valid under pose estimation error).
            if check_robust:
                for _ in range(10):
                    x, y, z = pose.pose[0]
                    new_pose = ((x + np.random.randn()*0.02, y + np.random.randn()*0.02, z), pose.pose[1])
                    new_pose = pb_robot.vobj.BodyPose(body, new_pose)
                    valid = fn(body, pose, grasp, check_robust=False)
                    if not valid:
                        print('Grasp not robust')
                        print(x - new_pose.pose[0][0], y - new_pose.pose[0][1])
                        return None

            if False:# and check_robust:
                length, lifeTime = 0.2, 0.0

                pos, quat = pb_robot.geometry.pose_from_tform(approach_tform)
                new_x = transformation([length, 0.0, 0.0], pos, quat)
                new_y = transformation([0.0, length, 0.0], pos, quat)
                new_z = transformation([0.0, 0.0, length], pos, quat)

                p.addUserDebugLine(pos, new_x, [1,0,0], lifeTime=lifeTime, physicsClientId=1)
                p.addUserDebugLine(pos, new_y, [0,1,0], lifeTime=lifeTime, physicsClientId=1)
                p.addUserDebugLine(pos, new_z, [0,0,1], lifeTime=lifeTime, physicsClientId=1)

            command = (pb_robot.vobj.MoveToTouch(robot.arm, q_approach, q_grasp, grasp, body, use_wrist_camera),
                       grasp,
                       pb_robot.vobj.MoveFromTouch(robot.arm, q_backoff, use_wrist_camera=use_wrist_camera))

            if return_grasp_q:
                return (pb_robot.vobj.BodyConf(robot, q_grasp),)
            # return (conf_approach, conf_backoff, command)
            return (conf_approach, command)
        return None
    return fn


def assign_fluent_state(fluents):
    obstacles = []
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        if name == 'atpose':
            o, p = args
            p = p.value
            o = o.value
            obstacles.append(o)
            continue
            o.set_base_link_pose(p.pose)
        else:
            raise ValueError(name)
    return obstacles


def get_free_motion_gen(robot, fixed=[], seed=None):
    def fn(conf1, conf2, fluents=[], seed=None):
        obstacles = assign_fluent_state(fluents)
        # print(fluents)
        # print(conf1, conf2, fluents, seed)
        # print(obstacles)
        # assert False
        fluent_names = [o.get_name() for o in obstacles]
        for o in fixed:
            if o.get_name() not in fluent_names:
                obstacles.append(o)

        # print(f'robot: {robot.get_custom_limits(robot.arm.joints)}')
        path = robot.arm.birrt.PlanToConfiguration(robot.arm, conf1.configuration, conf2.configuration, inflate=False, obstacles=obstacles)

        if path is None:
            if DEBUG_FAILURE: input('Free motion failed')
            return None
        command = (pb_robot.vobj.JointSpacePath(robot.arm, path),)
        return (command,)
    return fn


def get_holding_motion_gen(robot, fixed=[]):
    def fn(conf1, conf2, body, grasp, fluents=[], seed=None):
        obstacles = assign_fluent_state(fluents)
        fluent_names = [o.get_name() for o in obstacles]
        for o in fixed:
            if o.get_name() not in fluent_names:
                obstacles.append(o)
        print('jerererer')
        print(fluents)
        print(conf1, conf2, fluents, seed)
        print(obstacles)

        old_q = robot.arm.GetJointValues()
        orig_pose = body.get_base_link_pose()
        robot.arm.SetJointValues(conf1.configuration)
        robot.arm.Grab(body, grasp.grasp_objF)

        path = robot.arm.birrt.PlanToConfiguration(robot.arm, conf1.configuration, conf2.configuration, obstacles=obstacles, inflate=False)

        robot.arm.Release(body)
        body.set_base_link_pose(orig_pose)
        robot.arm.SetJointValues(old_q)

        if path is None:
            print(obstacles)
            if DEBUG_FAILURE: input('Holding motion failed')
            return None
        command = (pb_robot.vobj.JointSpacePath(robot.arm, path),)
        return (command,)
    return fn


def get_movable_collision_test(robot):
    def test(command, body, pose):
        body.set_base_link_pose(pose.pose)
        obstacles = [body]
        print('Checking collisions!\n\n\n')
        print(command)
        for motion in command:
            if type(motion) != pb_robot.vobj.JointSpacePath: continue
            for q in motion.path:
                if not robot.arm.IsCollisionFree(q, obstacles=obstacles):
                    if DEBUG_FAILURE: input('Movable collision')
                    return False
                print('HERE')
        return True
    return test