import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import lab10_map
import particle_filter_01
import rrt
import kinematics
import numpy as np

localize = False  # testing


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
        self.pf = particle_filter_01.ParticleFilter(the_map=self.map_localize)

        self.joint_angles = np.zeros(7)

        self.kinematics = kinematics.Kinematics(self.time)

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
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
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
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
            # self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
        self.create.drive_direct(0, 0)
        self.pf.turn(self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
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
        self.pf.forward(math.sqrt((self.odometry.x - old_x) ** 2 + (self.odometry.y - old_y) ** 2))

    def visualize(self):
        self.pf.draw(self.virtual_create)

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




        if localize:
            # Localize
            angle = math.pi / 15
            counter = 0
            while (np.array(self.pf.variance()) > np.array([0.01, 0.01])).any():
                distance = self.sonar.get_distance()
                self.go_to_angle(self.odometry.theta + angle)

                # self.odometry.theta += math.radians(2.54458909216)  # 48.88398233641108 actual ~ 51.4285714286 theory per turn

                print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
                self.pf.sense(distance)
                self.visualize()
                counter += 1
                self.time.sleep(0.01)

            # self.odometry.theta += math.radians(counter * 2.54458909216)  # 48.88398233641108 actual ~ 51.4285714286 theory

            # # trying to improve localization by turning one extra time
            # distance = self.sonar.get_distance()
            # self.go_to_angle(self.odometry.theta + angle)
            # print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
            # self.pf.sense(distance)
            # self.visualize()

            location = self.pf.mean()
        else:
            angle = 0
            location = [1, 0.5]

        self.odometry.x = location[0]
        self.odometry.y = location[1]
        # self.odometry.theta =
        # print('one more rotation')
        self.go_to_angle(self.odometry.theta + angle)
        # input("input: [{},{}, {}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
        # --------------------------------------Path Finder----------------------------------------

        # find a path
        print("self.rrt.build({}, {})".format(location[0] * 100, 300 - location[1] * 100))
        self.rrt.build((location[0] * 100, 300 - location[1] * 100), 300, 10)
        x_goal = (152, 66)
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

        for p in path:
            goal_x = p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0
            print("goal: {} {}".format(goal_x, goal_y))
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                    # move virtual robot
                    self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
                    # print(output_theta)
                    # print("Robot's coordinates[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    # print("Actual coordinates[{},{},{}]".format(self.odometry.y, -self.odometry.x, math.degrees(self.odometry.theta)-90))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.05:
                        break
        self.create.drive_direct(0, 0)
        self.time.sleep(2)
        # go to a specified angle at the end of the path
        self.go_to_angle(-np.pi/2)
        self.time.sleep(2)
        # --------------------------------------Wait ARM part----------------------------------------
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
        print("Robot at [%.3f, %.3f]" % (self.odometry.x, self.odometry.y))
        self.kinematics.pick_up_cup(self.arm, self.odometry.x, self.odometry.y)
        # input("wait for arm")
        self.kinematics.go_to_level0(self.arm)
        # self.kinematics.go_to_level1(self.arm)