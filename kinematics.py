import numpy as np

class Kinematics:
    def __init__(self, time):
        self.l1 = 0.4 # meters, distance from LBR4p_joint2 to LBR4p_joint4
        self.l2 = 0.4575 # meters, distance from LBR4p_joint4 to LBR4p_joint7 (end effector)
        self.calibration = 0.3105 # meters, distance from LBR4p_joint2 to ground
        self.curr_theta1 = 0
        self.curr_theta2 = 0
        self.time = time

    def go_to_level2(self, arm):
        arm.go_to(0, np.pi/4)
        self.time.sleep(2)
        arm.go_to(5, -np.pi/2.8)
        self.time.sleep(2)
        self.inverse_kinematics(arm, x = 0.45, z = 1)
        arm.go_to(0, -np.pi/18)

    def go_to_level3(self, arm):
        arm.go_to(0, np.pi/18)
        self.time.sleep(2)
        arm.go_to(5, -np.pi/4)
        self.time.sleep(2)
        self.inverse_kinematics(arm, x = 0.3, z = 1.1)

    def inverse_kinematics(self, arm, x, z):
        '''
        Given position of end effector, compute angle of theta1 and theta2
        '''
        r = np.sqrt(x**2 + (z-self.calibration)**2)
        alpha = np.arccos((self.l1**2 + self.l2**2 - r**2)/(2 * self.l1 * self.l2)) # [0, pi]
        beta = np.arccos((r**2 + self.l1**2 - self.l2**2)/(2 * self.l1 * r)) # [0, pi]
        phi = np.arctan2(x, z-self.calibration)
        theta1s = -1 * np.array([phi + beta, phi - beta])
        theta2s = np.array([np.pi - alpha, alpha - np.pi])

        # choose the answer with shorter distance
        ans_idx = 0
        if np.abs(theta1s[0] - self.curr_theta1) + np.abs(theta2s[0] - self.curr_theta2) > np.abs(theta1s[1] - self.curr_theta1) + np.abs(theta2s[1] - self.curr_theta2):
            ans_idx = 1

        print("Go to [%.3f, %.3f], IK: [%.3f deg, %.3f deg]" % (x, z, np.degrees(theta1s[ans_idx]), np.degrees(theta2s[ans_idx])))

        arm.go_to(1, theta1s[ans_idx])
        self.time.sleep(0.5)
        arm.go_to(3, theta2s[ans_idx])
        self.time.sleep(0.5)

        self.curr_theta1 = theta1s[ans_idx]
        self.curr_theta2 = theta2s[ans_idx]