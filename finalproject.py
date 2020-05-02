import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import kinematics
import particle_filter_01
import numpy as np


real = False  # real robot vs simulation
localize = True  # testing

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

        if not real:
            self.arm = factory.create_kuka_lbr4p()

        self.virtual_create = factory.create_virtual_create()

        self.odometry = odometry.Odometry()
        self.map_local = lab8_map.Map("lab8_map.json")  # partical filter localize
        self.map_path = lab10_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map_path)
        self.kinematics = kinematics.Kinematics(self.time)

        # TODO identify good PID controller gains
        # self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter_01.ParticleFilter(the_map=self.map_local)

        self.joint_angles = np.zeros(7)

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
            if distance < 0.1:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.forward(math.sqrt((self.odometry.x - old_x) ** 2 + (self.odometry.y - old_y) ** 2))

    def visualize(self):
        self.pf.draw(self.virtual_create)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            pyCreate2.Sensor.LeftEncoderCounts,
            pyCreate2.Sensor.RightEncoderCounts,
        ])
        self.visualize()
        self.virtual_create.enable_buttons()
        self.visualize()

        print("start grabing")
        self.kinematics.pick_up_cup(self.arm, 1.52, 2.35)
        self.kinematics.go_to_level0(self.arm)
        input("End")

        # while True:
        #     b = self.virtual_create.get_last_button()
        #     if b == self.virtual_create.Button.MoveForward:
        #         self.forward()
        #         self.visualize()
        #     elif b == self.virtual_create.Button.TurnLeft:
        #         self.go_to_angle(self.odometry.theta + math.pi / 2)
        #         self.visualize()
        #     elif b == self.virtual_create.Button.TurnRight:
        #         self.go_to_angle(self.odometry.theta - math.pi / 2)
        #         self.visualize()
        #     elif b == self.virtual_create.Button.Sense:
        #         distance = self.sonar.get_distance()
        #         print(distance)
        #         self.pf.measure(distance, 0)
        #         self.visualize()

        #     self.time.sleep(0.01)