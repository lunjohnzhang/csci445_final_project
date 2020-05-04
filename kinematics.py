import numpy as np
import math

class Kinematics:
    def __init__(self, time):
        # params for inverse kinematics
        self.l1 = 0.4 # meters, distance from LBR4p_joint2 to LBR4p_joint4
        self.l2 = 0.4575 # meters, distance from LBR4p_joint4 to LBR4p_joint7 (end effector)
        self.calibration = 0.3105 # meters, distance from LBR4p_joint2 to ground

        # angle of some relevant joints
        self.curr_theta1 = 0
        self.curr_theta3 = 0
        self.curr_theta5 = 0
        self.time = time

        # position of the joints controlled by inverse kinematics
        self._x = 0
        self._z = 0

        # corrdinate of the arm in meters

    def set_gripper(self, arm):
        '''
        Function to set the angle of gripper so that it is always parallel with the ground
        '''
        delta_cali = np.pi/2 - (self.curr_theta1 + self.curr_theta3)
        arm.go_to(5, delta_cali)
        self.curr_theta5 = delta_cali

    def get_move_range(self, _range, _step_size):
        '''
        Helper function to get the range of motion
        param:
            _range: shape (start, end), start and end of the range
            _step_size: how much to move at each step
        '''
        move_range = []
        if _range[0] <= _range[1]:
            move_range = [i for i in np.arange(_range[0], _range[1], _step_size)]
        else:
            move_range = [i for i in np.arange(_range[0], _range[1], -_step_size)]
        return move_range

    def step_go_to(self, arm, joint_idx, angle_range, step_size):
        '''
        Function to slowly move specified joints of the arm to move
        from current angle to specified angle
        param:
            arm: arm object to move
            joint_idxs: array of int that specifies which joints to move
            angle_range: shape (start, end) start and end of the angle in degrees
            step_size: how much angle in degrees to move at each step, assume to be positive
        '''
        move_range = self.get_move_range(angle_range, step_size)
        for i in move_range:
            # for joint_idx in joint_idxs:
            arm.go_to(joint_idx, np.radians(i))
            self.time.sleep(0.1)
        if joint_idx == 1:
            self.curr_theta1 = move_range[-1]
        elif joint_idx == 3:
            self.curr_theta3 = move_range[-1]
        elif joint_idx == 5:
            self.curr_theta5 = move_range[-1]

    def step_inv_kinematics(self, arm, corr_range, step_size, axis, const_corr):
        '''
        Function to slowly move arm using inverse kinematics
        param:
            arm: arm object to move
            corr_range: shape (start, end) start and end of the corrdinate to move in degrees
            step_size: how much angle in degrees to move at each step, assume to be positive
            axis: axis of the corrdinate to move, x or z
            const_corr: coordinate of the axis that does not move
        '''
        move_range = self.get_move_range(corr_range, step_size)
        for delta in move_range:
            if axis == "x":
                self.inverse_kinematics(arm, delta, const_corr)
            elif axis == "z":
                self.inverse_kinematics(arm, const_corr, delta)
            self.set_gripper(arm)

    def pick_up_cup(self, arm, curr_x, curr_y, map_idx):
        '''
        Function to slowly approach the arm and grab it
        params:
            arm: arm object
            curr_x: current x position of the robot under odometry coordinates
            curr_y: current y position of the robot under odometry coordinates
            map_idx: which map is in use. 1 for simple once, 2 for complex once.
        '''
        # origin of two of the maps are different...
        if map_idx == 1:
            self.arm_x = 1.6001
            self.arm_y = 3.3999
            delta_theta = np.arctan2(curr_x - self.arm_x, self.arm_y - curr_y)
            forward_cali = 0.28
        elif map_idx == 2:
            self.arm_x = -0.3999
            self.arm_y = 1.6000
            delta_theta = np.arctan2(curr_y - self.arm_y, curr_x - self.arm_x)
            delta_theta += np.radians(5)
            forward_cali = 0.30

        arm.open_gripper()
        self.time.sleep(5)

        # calculate delta theta with the robot arm
        print("delta_theta = %.3f" % (np.degrees(delta_theta)))
        arm.go_to(0, delta_theta)

        # calculate delta dist with the robot arm
        delta_dist = np.linalg.norm(np.array([curr_x, curr_y]) - np.array([self.arm_x, self.arm_y]))
        print("delta_dist = %.3f" % delta_dist)

        # move a position behind the robot
        self.inverse_kinematics(arm, x = -delta_dist+0.5, z = 0.15) # plus a constant for calibration
        self.set_gripper(arm)

        # slowly move to the cup and grab it
        self.step_inv_kinematics(arm, (self._x, -delta_dist+forward_cali), 0.001, "x", self._z)
        arm.close_gripper()
        self.time.sleep(10)

        # sloly move the arm back a little bit
        self.step_inv_kinematics(arm, (self._x, -delta_dist+0.5), 0.001, "x", self._z)

    def go_to_level0(self, arm):
        # slowly move the arm up
        self.step_inv_kinematics(arm, (self._z, 0.3), 0.001, "z", self._x)
        self.time.sleep(10)

        # rotate joint0 to specified angle
        self.step_go_to(arm, 0, (0, -126), 1)

        # put down the cup
        self.step_inv_kinematics(arm, (self._z, 0.15), 0.001, "z", self._x)
        arm.open_gripper()
        self.time.sleep(5)

    def go_to_level1(self, arm):
        # slowly move the arm up
        self.step_inv_kinematics(arm, (self._z, 0.65), 0.001, "z", self._x)
        self.time.sleep(10)

        # rotate joint0 to specified angle
        self.step_go_to(arm, 0, (0, -136), 1)

        # put down the cup
        self.step_inv_kinematics(arm, (self._z, 0.55), 0.001, "z", self._x)
        arm.open_gripper()
        self.time.sleep(5)

    def go_to_level2(self, arm):
        # slowly move the arm up
        self.step_inv_kinematics(arm, (self._z, 0.65), 0.001, "z", self._x)
        self.time.sleep(10)

        # move the arm back a little more
        self.step_inv_kinematics(arm, (self._x, -0.3), 0.001, "x", self._z)

        # move the arm up more
        self.step_inv_kinematics(arm, (self._z, 1), 0.001, "z", self._x)

        # rotate joint0 to specified angle
        self.step_go_to(arm, 0, (0, -150), 1)

        # put down the cup
        self.step_inv_kinematics(arm, (self._z, 0.90), 0.001, "z", self._x)
        arm.open_gripper()
        self.time.sleep(5)

    def go_to_level3(self, arm, map_idx = 1):
        # slowly move the arm up
        self.step_inv_kinematics(arm, (self._z, 0.65), 0.001, "z", self._x)
        self.time.sleep(10)

        # move the arm back more
        self.step_inv_kinematics(arm, (self._x, 0), 0.001, "x", self._z)

        # move the arm up more
        self.step_inv_kinematics(arm, (self._z, 1.15), 0.001, "z", self._x)

        # rotate joint0 to specified angle
        self.step_go_to(arm, 0, (0, -170), 1)

        print("curr theta5 = %.3f" % np.degrees(self.curr_theta5))

        # input("here")

        # uprise the gripper
        if map_idx == 1:
            self.step_go_to(arm, 5, (np.degrees(self.curr_theta5), 60), 1)
        elif map_idx == 2:
            self.step_go_to(arm, 5, (np.degrees(self.curr_theta5), 45), 1)

        # input("Here")

        self.step_go_to(arm, 1, (self.curr_theta1, 45), 1)

        # # move the arm forward a little
        # self.step_inv_kinematics(arm, (self._x, -0.2), 0.001, "x", self._z)

        # # put down the cup
        # self.step_inv_kinematics(arm, (self._z, 1.1), 0.001, "z", self._x)
        arm.open_gripper()
        self.time.sleep(5)

    def inverse_kinematics(self, arm, x, z):
        '''
        Given position of end effector, compute angle of theta1 and theta2
        '''
        r = np.sqrt(x**2 + (z-self.calibration)**2)
        print("r = %.3f" % r)
        alpha = np.arccos((self.l1**2 + self.l2**2 - r**2)/(2 * self.l1 * self.l2)) # [0, pi]
        print("cos(alpha) = %.3f" % ((self.l1**2 + self.l2**2 - r**2)/(2 * self.l1 * self.l2)))
        beta = np.arccos((r**2 + self.l1**2 - self.l2**2)/(2 * self.l1 * r)) # [0, pi]
        print("cos(beta) = %.3f" % ((r**2 + self.l1**2 - self.l2**2)/(2 * self.l1 * r)))
        phi = np.arctan2(x, z-self.calibration)
        theta1s = -1 * np.array([phi + beta, phi - beta])
        theta2s = np.array([np.pi - alpha, alpha - np.pi])

        # choose the answer with shorter distance
        ans_idx = 0
        if np.abs(theta1s[0] - self.curr_theta1) + np.abs(theta2s[0] - self.curr_theta3) > np.abs(theta1s[1] - self.curr_theta1) + np.abs(theta2s[1] - self.curr_theta3):
            ans_idx = 1

        print("Go to [%.3f, %.3f], IK: [%.3f deg, %.3f deg]" % (x, z, np.degrees(theta1s[ans_idx]), np.degrees(theta2s[ans_idx])))

        arm.go_to(1, theta1s[ans_idx])
        self.time.sleep(0.01)
        arm.go_to(3, theta2s[ans_idx])
        self.time.sleep(0.01)

        self.curr_theta1 = theta1s[ans_idx]
        self.curr_theta3 = theta2s[ans_idx]
        self._x = x
        self._z = z