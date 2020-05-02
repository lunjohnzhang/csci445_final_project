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
        self.arm_x = 1.6001
        self.arm_y = 3.3999

    def set_gripper(self, arm):
        '''
        Function to set the angle of gripper so that it is always parallel with the ground
        '''
        delta_cali = np.pi/2 - (self.curr_theta1 + self.curr_theta3)
        arm.go_to(5, delta_cali)
        self.curr_theta5 = delta_cali

    def pick_up_cup(self, arm, curr_x, curr_y):
        '''
        Function to slowly approach the arm and grab it

        params:
            arm: arm object
            curr_x: current x position of the robot under odometry coordinates
            curr_y: current y position of the robot under odometry coordinates
        '''
        arm.open_gripper()
        self.time.sleep(5)

        # calculate delta theta with the robot arm
        delta_theta = np.arctan2(curr_x - self.arm_x, self.arm_y - curr_y)
        print("delta_theta = %.3f" % (np.degrees(delta_theta)))
        arm.go_to(0, delta_theta)

        # calculate delta dist with the robot arm
        delta_dist = np.linalg.norm(np.array([curr_x, curr_y]) - np.array([self.arm_x, self.arm_y]))
        print("delta_dist = %.3f" % delta_dist)

        # move a position behind the robot
        self.inverse_kinematics(arm, x = -delta_dist+0.5, z = 0.15) # plus a constant for calibration
        self.set_gripper(arm)

        # slowly move to the cup and grab it
        self.step_inv_kinematics(arm, (self._x, -delta_dist+0.25), 0.001, "x", self._z)
        arm.close_gripper()
        self.time.sleep(10)

        # slowly move the arm up
        self.step_inv_kinematics(arm, (self._z, 0.3), 0.001, "z", self._x)
        self.time.sleep(20)

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

    def go_to_level0(self, arm):
        '''
        Go to level 0 of the shelf
        '''
        self.step_go_to(arm, 0, (0, -126), 1)
        self.step_inv_kinematics(arm, (self._z, 0.1), 0.001, "z", self._x)
        arm.open_gripper()
        self.time.sleep(5)

    def go_to_level1(self, arm):
        '''
        Go to level 1 of the shelf
        '''
        print("move to second floor")
        arm.go_to(0, math.radians(-90))
        arm.go_to(2, math.radians(-90))
        self.time.sleep(1)
        self.inverse_kinematics(arm, x=-0.7, z = 0.17)
        arm.go_to(4, math.radians(65))
        self.time.sleep(5)

        arm.open_gripper()
        self.time.sleep(5)
        arm.go_to(2, math.radians(-45))
        self.time.sleep(1)

        arm.go_to(0, math.radians(0))
        arm.go_to(1, math.radians(0))
        arm.go_to(2, math.radians(0))
        arm.go_to(3, math.radians(0))
        arm.go_to(4, math.radians(0))
        arm.go_to(5, math.radians(0))
        self.time.sleep(2)


    def go_to_level2(self, arm):
        '''
        Go to level 2 of the shelf
        '''
        arm.go_to(0, np.pi/2)
        self.time.sleep(2)
        arm.go_to(5, -np.pi/2.8)
        self.time.sleep(2)
        self.inverse_kinematics(arm, x = 0.45, z = 1)
        arm.go_to(0, np.pi/4)
        self.time.sleep(2)
        arm.go_to(0, -np.pi/18)
        arm.open_gripper()
        self.time.sleep(5)

        arm.go_to(0, np.pi/2)
        arm.go_to(0, math.radians(0))
        arm.go_to(1, math.radians(0))
        arm.go_to(3, math.radians(0))
        arm.go_to(5, math.radians(0))

    def go_to_level3(self, arm):
        '''
        Go to level 3 of the shelf
        '''
        arm.go_to(0, np.pi/18)
        self.time.sleep(2)
        arm.go_to(5, -np.pi/4)
        self.time.sleep(2)
        self.inverse_kinematics(arm, x = 0.3, z = 1.1)
        arm.open_gripper()
        self.time.sleep(5)

        arm.go_to(0, math.radians(0))
        arm.go_to(1, math.radians(0))
        arm.go_to(3, math.radians(0))
        arm.go_to(5, math.radians(0))

    def inverse_kinematics(self, arm, x, z):
        '''
        Given position of end effector, compute angle of theta1 and theta3
        '''
        r = np.sqrt(x**2 + (z-self.calibration)**2)
        print("r = %.3f" % r)
        alpha = np.arccos((self.l1**2 + self.l2**2 - r**2)/(2 * self.l1 * self.l2)) # [0, pi]
        print("cos(alpha) = %.3f" % ((self.l1**2 + self.l2**2 - r**2)/(2 * self.l1 * self.l2)))
        beta = np.arccos((r**2 + self.l1**2 - self.l2**2)/(2 * self.l1 * r)) # [0, pi]
        print("cos(beta) = %.3f" % ((r**2 + self.l1**2 - self.l2**2)/(2 * self.l1 * r)))
        phi = np.arctan2(x, z-self.calibration)
        theta1s = -1 * np.array([phi + beta, phi - beta])
        theta3s = np.array([np.pi - alpha, alpha - np.pi])

        # choose the answer with shorter distance
        ans_idx = 0
        if np.abs(theta1s[0] - self.curr_theta1) + np.abs(theta3s[0] - self.curr_theta3) > np.abs(theta1s[1] - self.curr_theta1) + np.abs(theta3s[1] - self.curr_theta3):
            ans_idx = 1

        print("Go to [%.3f, %.3f], IK: [%.3f deg, %.3f deg]" % (x, z, np.degrees(theta1s[ans_idx]), np.degrees(theta3s[ans_idx])))

        arm.go_to(1, theta1s[ans_idx])
        self.time.sleep(0.01)
        arm.go_to(3, theta3s[ans_idx])
        self.time.sleep(0.01)

        self.curr_theta1 = theta1s[ans_idx]
        self.curr_theta3 = theta3s[ans_idx]
        self._x = x
        self._z = z