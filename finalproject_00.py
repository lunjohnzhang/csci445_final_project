import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import lab10_map
import particle_filter
import rrt
import kinematics
import numpy as np
import time

rack = 1

cup_x = -1
cup_y = -1  # 2.31
cup_z = 0.19  #0.16167
arm_x = 1.6
arm_y = 3.4
gripper_offset = 0.2837  # 0.3  # 0.2837

# heights = [0.1617, 0.5367, 0.8367, 1.1367, 1.4367]  # lowest
heights = [0.2367, 0.6367, 0.9367, 1.2367, 1.5367]  # highest
# highest level: 0.3367, 0.6367, (+0.1,etc, except lowest rack)
reach = 0.5801  # distance between gripper and shelf

class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()

        self.arm = factory.create_kuka_lbr4p()
        self.kinematics = kinematics.Kinematics(self.time)

        self.virtual_create = factory.create_virtual_create()
        #self.virtual_create = factory.create_virtual_create("192.168.1.177")

        self.odometry = odometry.Odometry()
        self.map_localize = lab8_map.Map("lab8_map.json")  # partical filter localize
        self.map_path = lab10_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map_path)

        # TODO identify good PID controller gains
        # self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.map, 1000, 0.01, 0.05, 0.1)

        self.joint_angles = np.zeros(7)

        self.kinematics = kinematics.Kinematics(self.time)

    # --------------------------------------Helper Method----------------------------------------
    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.05:
            # print("Go TO: " + str(goal_theta) + " " + str(self.odometry.theta))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        print(goal_x, goal_y)
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def take_measurements(self):

    # angle = -90
    #     while angle <= 90:
    #         self.servo.go_to(angle)
    #         self.time.sleep(2.0)
    #         distance = self.sonar.get_distance()
    #         print(distance)
    #         self.pf.measure(distance, math.radians(angle))
    #         self.map.draw(self.pf, "test{}.png".format(self._pos))
    #         x, y, theta = self.pf.get_estimate()
    #         self.virtual_create.set_pose((x, y, 0.1), theta)
    #
    #         data = []
    #         for particle in self.pf.particles_for_map:
    #             data.extend([particle.x, particle.y, 0.1, particle.theta])
    #
    #         self.virtual_create.set_point_cloud(data)
    #
    #         angle += 45
    #         self._pos += 1

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def ik(self, x_i, z_i):
        L1 = 0.4  # estimated using V-REP (joint2 - joint4)
        L2 = 0.39  # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        # x = -x_i
        x = x_i

        # compute inverse kinematics
        r = math.sqrt(x * x + z * z)
        print('x', x)
        print('z', z)
        print("r", r)
        print((L1 * L1 + L2 * L2 - r * r))
        print((2 * L1 * L2))

        try:
            alpha = math.acos((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))
        except ValueError:
            print("alpha ValueError: math domain error")
            return
        theta2 = math.pi - alpha

        try:
            beta = math.acos((r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))
        except ValueError:
            print("beta ValueError: math domain error")
            return

        print('alpha deg', math.degrees(alpha))
        print('beta deg', math.degrees(beta))

        theta1 = math.atan2(x, z) - beta

        print('t1:', math.degrees(theta1))
        print('t2:', math.degrees(theta2))

        return [theta1, theta2, math.pi / 2 - theta1 - theta2]

    def run(self):

        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        self.time.sleep(4)

        # request sensors
        self.create.start_stream([
            pyCreate2.Sensor.LeftEncoderCounts,
            pyCreate2.Sensor.RightEncoderCounts,
        ])
        self.visualize()



        # Localize
        angle = math.pi / 3
        while (np.array(self.pf.variance()) > np.array([0.02, 0.02])).any():
            distance = self.sonar.get_distance()
            self.go_to_angle(self.odometry.theta + angle)
            print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
            self.pf.sense(distance)
            self.visualize()
            self.time.sleep(0.01)
        location = self.pf.mean()

        # self.odometry.theta += math.radians(counter * 2.54458909216)  # 48.88398233641108 actual ~ 51.4285714286 theory

        # # trying to improve localization by turning one extra time
        # self.go_to_angle(self.odometry.theta + angle)
        # print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
        # self.pf.sense(distance)
        # self.visualize()


        #location = self.pf.get_estimate()
        #location = [1, 0.5]

        self.odometry.x = location[0]
        self.odometry.y = location[1]
        # self.odometry.theta = location[2]
        # print('one more rotation')
        #self.go_to_angle(self.odometry.theta + angle)
        # input("input: [{},{}, {}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
        # --------------------------------------Path Finder----------------------------------------

        # find a path
        print("self.rrt.build({}, {})".format(location[0] * 100, 300 - location[1] * 100))
        self.rrt.build((location[0] * 100, 300 - location[1] * 100), 300, 10)
        x_goal = (155, 20)
        x_goal_nn = self.rrt.nearest_neighbor(x_goal)
        path = self.rrt.shortest_path(x_goal_nn)
        path.append(rrt.Vertex(x_goal))

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map_path.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
        for idx in range(0, len(path) - 1):
            self.map_path.draw_line((path[idx].state[0], path[idx].state[1]),
                               (path[idx + 1].state[0], path[idx + 1].state[1]), (0, 255, 0))

        self.map_path.save("configspace_rrt.png")

        # input('input: path found and configspace_rrt.png saved')
        # --------------------------------------Move----------------------------------------
        base_speed = 100


        check_time = time.time()
        check_end = True
        for p in path:
            goal_x = p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0
            print("goal: {} {}".format(goal_x, goal_y))
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    #theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                    # move virtual robot
                    #self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
                    # print(output_theta)
                    # print("Robot's coordinates[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    # print("Actual coordinates[{},{},{}]".format(self.odometry.y, -self.odometry.x, math.degrees(self.odometry.theta)-90))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

                    if distance < 0.05:
                        break
        self.create.drive_direct(0, 0)
        self.time.sleep(2)

        cup_x = self.odometry.x
        cup_y = self.odometry.y
        print('cup at', cup_x, cup_y)

        self.arm.open_gripper()

        print('START grabbing cup')
        a0 = math.atan2(-(arm_x - cup_x), arm_y - cup_y)
        self.arm.go_to(0, a0)
        old_a0 = a0

        old_a1 = math.radians(0)
        old_a2 = math.radians(105)
        old_a3 = math.pi / 2 - old_a1 - old_a2
        self.arm.go_to(1, old_a1)
        self.arm.go_to(3, old_a2)
        self.arm.go_to(5, old_a3)
        self.sleep(2)

        end_z = cup_z
        end_x = math.sqrt((arm_x - cup_x) ** 2 + (arm_y - cup_y) ** 2) - gripper_offset
        a1, a2, a3 = self.ik(end_x, end_z)

        size = 100
        for i in range(size):
            self.arm.go_to(1, old_a1 + i / size * (a1 - old_a1))
            self.arm.go_to(3, old_a2 + i / size * (a2 - old_a2))
            self.arm.go_to(5, old_a3 + i / size * (a3 - old_a3))
            self.sleep(0.01)
        self.arm.go_to(1, a1)
        self.arm.go_to(3, a2)
        self.arm.go_to(5, a3)
        self.sleep(1)

        self.arm.close_gripper()
        self.sleep(5)
        # start_z = end_z
        # start_x = end_x
        old_a1 = a1
        old_a2 = a2
        old_a3 = a3
        print('END grabbing cup')

        print('START lifting cup to midway')
        # rack = 2
        # end_z = heights[rack]
        end_z = 0.6
        end_x = 0.5  # reach
        a1, a2, a3 = self.ik(end_x, end_z)
        print(math.degrees(a1), math.degrees(a2), math.degrees(a3))
        size = 100
        for i in range(size):
            # print(i)
            self.arm.go_to(1, old_a1 + i / size * (a1 - old_a1))
            self.arm.go_to(3, old_a2 + i / size * (a2 - old_a2))
            self.arm.go_to(5, old_a3 + i / size * (a3 - old_a3))
            self.sleep(0.01)
        self.arm.go_to(1, a1)
        self.arm.go_to(3, a2)
        self.arm.go_to(5, a3)
        self.sleep(1)
        old_a1 = a1
        old_a2 = a2
        old_a3 = a3
        print('END lifting cup to midway')

        print('START rotate whole arm')
        a0 = -math.pi / 2
        for i in range(int(math.degrees(old_a0)), int(math.degrees(a0)), -1):
            self.arm.go_to(0, math.radians(i))
            self.sleep(0.01)
        old_a0 = a0
        self.sleep(1)
        print('END rotate whole arm')

        print('START moving to rack height')

        end_z = heights[rack]
        end_x = reach - gripper_offset
        a1, a2, a3 = self.ik(end_x, end_z)
        print(math.degrees(a1), math.degrees(a2), math.degrees(a3))
        size = 100
        for i in range(size):
            # print(i)
            self.arm.go_to(1, old_a1 + i / size * (a1 - old_a1))
            self.arm.go_to(3, old_a2 + i / size * (a2 - old_a2))
            self.arm.go_to(5, old_a3 + i / size * (a3 - old_a3))
            self.sleep(0.01)
        self.arm.go_to(1, a1)
        self.arm.go_to(3, a2)
        self.arm.go_to(5, a3)
        self.sleep(1)
        old_a1 = a1
        old_a2 = a2
        old_a3 = a3
        print('END moving to rack height')

        print('START final rotation')
        a0 = -math.pi
        for i in range(int(math.degrees(old_a0)), int(math.degrees(a0)), -1):
            self.arm.go_to(0, math.radians(i))
            self.sleep(0.01)
        old_a0 = a0
        self.sleep(1)
        self.arm.open_gripper()
        self.sleep(5)
        print('END final rotation')

        print('START go back')
        self.arm.go_to(0, math.radians(-90))
        self.sleep(1)
        self.arm.go_to(0, math.radians(0))
        self.arm.go_to(1, math.radians(0))
        self.arm.go_to(3, math.radians(0))
        self.arm.go_to(5, math.radians(0))
        self.sleep(1)
        print('END go back')


        # go to a specified angle at the end of the path
        self.go_to_angle(-np.pi/2)
        self.time.sleep(2)
        # --------------------------------------Wait ARM part----------------------------------------
        '''
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
        print("Robot at [%.3f, %.3f]" % (self.odometry.x, self.odometry.y))
        self.kinematics.pick_up_cup(self.arm, self.odometry.x, self.odometry.y)
        input("wait for arm")
        self.kinematics.go_to_level0(self.arm)
        self.kinematics.go_to_level1(self.arm)
        '''