import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import lab10_map_2
import particle_filter_01
import rrt
import numpy as np
import kinematics

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
        self.map_local = lab8_map.Map("final_map.json")  # partical filter localize
        self.map_path = lab10_map_2.Map("configuration_space_2.png")
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

    def visualize(self):
        self.pf.draw(self.virtual_create)

    def sense_helper_complex(self):
        '''
        Function to help localize in complex map
        '''
        servo_angles = [90, 45, 0, -45, -90]
        for servo_angle in servo_angles:
            self.servo.go_to(servo_angle)
            self.time.sleep(1)
            distance = self.sonar.get_distance()
            # self.go_to_angle(self.odometry.theta + angle)
            # print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
            self.pf.sense(distance, np.radians(servo_angle))
            self.visualize()
            self.time.sleep(0.01)

    def go_to_loc(self, goal_x, goal_y, base_speed):
        '''
        Function for the robot go to a position
        '''
        while True:
            state = self.create.update()

            if state is not None:
                print("Goal: [%.3f, %.3f]" % (goal_x, goal_y))
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                if distance < 0.07:
                    break

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
            while (np.array(self.pf.variance()) > np.array([0.005, 0.005])).any():
                distance = self.sonar.get_distance()
                self.go_to_angle(self.odometry.theta + angle)
                print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
                self.pf.sense(distance)
                self.visualize()
                self.time.sleep(0.01)
            location = self.pf.mean()
        else:
            location = [1, 0.5]

        self.odometry.x = location[0]
        self.odometry.y = location[1]
        self.odometry.theta = self.odometry.theta + math.pi / 2

        # print('one more rotation')
        # self.go_to_angle(self.odometry.theta + angle)
        # input("input: [{},{}, {}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
        # --------------------------------------ADDED>>----------------------------------------

        # find a path
        # input("self.rrt.build({}, {})".format(location[0] * 100, 300 - location[1] * 100))
        # self.rrt.build((location[0] * 100, 300 - location[1] * 100), 3000, 10)
        self.rrt.build((location[0] * 100, 300 - location[1] * 100), 1000, 10)
        x_goal = (60, 152)
        x_goal_nn = self.rrt.nearest_neighbor(x_goal)
        path = self.rrt.shortest_path(x_goal_nn)
        path.append(rrt.Vertex(x_goal))

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map_path.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
        for idx in range(0, len(path) - 1):
            self.map_path.draw_line((path[idx].state[0], path[idx].state[1]),
                               (path[idx + 1].state[0], path[idx + 1].state[1]), (0, 255, 0))

        self.map_path.save("configspace_rrt_sim.png")

        # --------------------------------------ADDED>>----------------------------------------
        base_speed = 100

        step_check = 1
        for p in path:

            #self.real_odometry_x = 0.0
            #self.real_odometry_y = 0.0

            goal_x = p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0

            #self.real_goal_x = goal_x + (goal_y - standard_y)
            #self.real_goal_y = goal_y - (goal_x - standard_x)

            old_x = self.odometry.x
            old_y = self.odometry.y
            old_theta = self.odometry.theta

            self.go_to_loc(goal_x, goal_y, base_speed)


            self.sleep(0.01)
            self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
            self.visualize()
            step_check = step_check+1

            #check every 10 steps
            if step_check%10 is 0:
                angle = math.pi / 3
                while (np.array(self.pf.variance()) > np.array([0.01, 0.01])).any():
                    distance = self.sonar.get_distance()
                    self.go_to_angle(self.odometry.theta + angle)
                    print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
                    self.pf.sense(distance)
                    self.visualize()
                    self.time.sleep(0.01)
                location = self.pf.mean()
                self.odometry.x = location[0]
                self.odometry.y = location[1]




        self.create.drive_direct(0, 0)
        self.time.sleep(10)
        # --------------------------------------<<ADDED----------------------------------------

        # input("Last Localization Check!")
        if not real:
            if localize:
                # Localize
                angle = math.pi / 15
                counter = 0
                while (np.array(self.pf.variance()) > np.array([0.005, 0.005])).any():
                    distance = self.sonar.get_distance()
                    self.go_to_angle(self.odometry.theta + angle)
                    self.pf.sense(distance)
                    self.visualize()
                    counter += 1
                    self.time.sleep(0.01)

                # trying to improve localization by turning one extra time
                distance = self.sonar.get_distance()
                self.go_to_angle(self.odometry.theta + angle)
                self.pf.sense(distance)
                self.visualize()
                location = self.pf.mean()

            self.odometry.x = location[0]
            self.odometry.y = location[1]
            # self.odometry.theta =
            # input("input: [{},{}, {}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
        print("start grabing")
        self.go_to_loc(x_goal[0]/100, 3 - x_goal[1]/100, base_speed) # attemp to go to the final location again after relocalication
        self.go_to_angle(0) # 0 odometry deg of robot changed
        state = self.create.update()
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        print("Robot at [%.3f, %.3f, %.3f]" % (self.odometry.x, self.odometry.y, np.degrees(self.odometry.theta)))
        # self.virtual_create.set_pose((self.odometry.x, self.odometry.y, 0.1), self.odometry.theta)
        self.kinematics.pick_up_cup(self.arm, self.odometry.x, self.odometry.y, map_idx = 2)
        self.kinematics.go_to_level0(self.arm, map_idx = 2)
        input("End")