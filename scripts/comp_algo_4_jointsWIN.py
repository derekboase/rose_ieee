#! /usr/bin/env python


import matplotlib.pyplot as plt
import numpy as np
import rospy
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
# from std_srvs.srv import Empty
# from random import uniform


class RandomTrajectory:
    def __init__(self, home_joints, freq=10.0, runtime=10.0):
        # All the setup stuff for the nodes and topics
        # self.topic_name = '/j2s6s200/effort_joint_trajectory_controller/command'  # Simulation
        self.topic_name = '/j2s6s200_driver/trajectory_controller/command'  # Real bot
        self.pub = rospy.Publisher(self.topic_name, JointTrajectory, queue_size=1)
        # Instantiation of messages and names
        self.jointCmd = JointTrajectory()
        self.point = JointTrajectoryPoint()
        self.jointCmd.joint_names = ['j2s6s200_joint_1',
                                     'j2s6s200_joint_2',
                                     'j2s6s200_joint_3',
                                     'j2s6s200_joint_4',
                                     'j2s6s200_joint_5',
                                     'j2s6s200_joint_6']

        # Setting initial values to the point message
        self.current_joints = home_joints
        self.point.positions = self.current_joints
        self.point.velocities = [0, 0, 0, 0, 0, 0]
        self.point.accelerations = [0, 0, 0, 0, 0, 0]
        self.point.effort = []

        # All the time related information
        self.Ts = 1/freq
        self.rate = rospy.Rate(freq)
        self.end_time = runtime

        # Joint trajectories and index

        # IMPROVE THIS BY MAKING IT ONE LARGE ARRAY CAllED self.traj with dimensions
        self.j1_traj = []
        self.j2_traj = []
        self.j3_traj = []
        self.j4_traj = []

        self.actual_positions = np.zeros((1, 6))
        self.actual_velocities = np.zeros((1, 6))

        self.idx = 0

        self.algorithm_j1 = []
        self.algorithm_j2 = []
        self.algorithm_j3 = []
        self.algorithm_j4 = []

    def trajectory_calculator(self):
        """
        This function calculates the joint space positions of the nominal trajectory given an equation
        :return: None
        """
        # Trajectory points
        _time = np.arange(0, self.end_time + self.Ts, step=self.Ts)
        _tau = self.end_time/6.0
        _amplitude = 3*np.pi/4
        for t in _time:
            # self.j1_traj.append(_amplitude * (1 - np.exp(-_tau * t / np.pi)))
            # if t <= self.end_time*4/5:
            if t <= self.end_time/2.0:
                self.j1_traj.append(_amplitude * (1 - np.exp(-t/_tau)))
                self.j3_traj.append(np.pi)

            else:
                self.j1_traj.append(_amplitude * np.exp(-(t - self.end_time/2)/_tau))
                self.j3_traj.append(4.0*np.pi/5.0)
            self.j4_traj.append(np.pi/2.0*np.sin(8.0*np.pi*t/self.end_time) - 3.0*np.pi/4.0)
        self.j2_traj = np.linspace(np.pi/2, 5.0*np.pi/6, num=len(_time))

    def actual_values(self, real_joint_angles):
        self.actual_positions = np.array(real_joint_angles.position[0:6])
        self.actual_velocities = np.array(real_joint_angles.velocity[0:6])
        # print('The current value of J1 is {}'.format(self.actual_positions[0]))

    def move_joint_home(self):
        """
        This function moves the manipulator arm to the
        :return:
        """
        time_to_home = 3.0
        self.rate = rospy.Rate(10.0)
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(time_to_home)
        self.jointCmd.points.append(self.point)
        count = 0
        while count < (time_to_home + 1)*1/0.1:  # Make sure it has enough time to find home position
            self.pub.publish(self.jointCmd)
            count = count + 1
            self.rate.sleep()
        self.rate = rospy.Rate(1/self.Ts)

    def update_target(self, next_targets):
        next_targets = np.array(next_targets)
        self.jointCmd.points = []
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(self.Ts)
        self.point.velocities = ((next_targets - np.array(self.current_joints)) / self.Ts).tolist()
        # self.point.accelerations = (np.array(self.point.velocities) / self.Ts).tolist()
        self.current_joints = next_targets.tolist()
        # self.point.positions = self.current_joints
        self.point.positions = next_targets.tolist()
        self.jointCmd.points.append(self.point)

    def phi_estimator(self, phi_k1, delta_u_k1, delta_y_k, PHI_INIT=0.01):
        phi_func = phi_k1 +(NU*delta_u_k1)/(MU + delta_u_k1**2)*(delta_y_k - phi_k1*delta_u_k1)
        if phi_func <= EPSILON or np.absolute(delta_u_k1) <= EPSILON or np.sign(phi_func) != np.sign(PHI_INIT):
            phi_func = PHI_INIT
        return phi_func

    def control_action_calculator(self, phi_func, u_arr, e_k):
        term_1 = phi_func**2/(LAM + phi_func**2)*u_arr[-1]
        sum = 0
        for idx in range(len(ALPHA)):
            sum += ALPHA[idx]*u_arr[idx]
        term_2 = LAM/(LAM + phi_func**2)*sum
        term_3 = (RHO*phi_func)/(LAM + phi_func**2)*e_k
        return term_1 + term_2 + term_3 

    def signal_update(self):
        global _k_lst, y, EPSILON, LAM, MU, NU, RHO, L, ALPHA
        
        EPSILON, LAM, MU, NU, RHO, L = 0.0001, 0.1, 0.01, 0.8, 0.8, 4
        ALPHA = [1/2.0, 1/4.0, 1/8.0, 1/8.0]

        PHI_INIT1 = 25
        PHI_INIT2 = 25
        PHI_INIT3 = 55
        PHI_INIT4 = 10

        _N = self.end_time / self.Ts

        next_tar = self.current_joints

        y1, y2, y3, y4 = [], [], [], []
        u1, u2, u3, u4 = [], [], [], []
        # For k = 0 
        _k = 0
        phi_k_joint1 = PHI_INIT1
        phi_k_joint2 = PHI_INIT2
        phi_k_joint3 = PHI_INIT3
        phi_k_joint4 = PHI_INIT4

        # Measured positions
        y1.append(self.actual_positions[0])
        y2.append(self.actual_positions[1])     
        y3.append(self.actual_positions[2])
        y4.append(self.actual_positions[3])

        # Error calculatiosn
        e_k_joint1 = self.j1_traj[_k + 1] - y1[-1]
        e_k_joint2 = self.j2_traj[_k + 1] - y2[-1]
        e_k_joint3 = self.j3_traj[_k + 1] - y3[-1]
        e_k_joint4 = self.j4_traj[_k + 1] - y4[-1]

        # Control action calculation
        u_k_joint_1 = self.control_action_calculator(phi_k_joint1, [0, 0, 0, 0], e_k_joint1)
        u1.append(u_k_joint_1)
        u_k_joint_2 = self.control_action_calculator(phi_k_joint2, [0, 0, 0, 0], e_k_joint2)
        u2.append(u_k_joint_2)
        u_k_joint_3 = self.control_action_calculator(phi_k_joint3, [0, 0, 0, 0], e_k_joint3)
        u3.append(u_k_joint_3)
        u_k_joint_4 = self.control_action_calculator(phi_k_joint4, [0, 0, 0, 0], e_k_joint4)
        u4.append(u_k_joint_4)

        # Target vector update
        next_tar[0] += u_k_joint_1
        next_tar[1] += u_k_joint_2
        next_tar[2] += u_k_joint_3
        next_tar[3] += u_k_joint_4

        self.update_target(next_tar)
        self.pub.publish(self.jointCmd)
        self.rate.sleep()
        # time.sleep(3.0/4.0*self.Ts) # Delay to match frequencies

        # For k = 1
        _k = 1
        # Delta u values 
        delta_u_1 = u1[-1] - 0
        delta_u_2 = u2[-1] - 0
        delta_u_3 = u3[-1] - 0
        delta_u_4 = u4[-1] - 0

        # Measured Positions
        y1.append(self.actual_positions[0])
        y2.append(self.actual_positions[1])
        y3.append(self.actual_positions[2])
        y4.append(self.actual_positions[3])

        # Delta y values
        delta_y_1 = y1[-1] - y1[-2]
        delta_y_2 = y2[-1] - y2[-2]
        delta_y_3 = y3[-1] - y3[-2]
        delta_y_4 = y4[-1] - y4[-2]

        # Error calculatiosn
        e_k_joint1 = self.j1_traj[_k + 1] - y1[-1]
        e_k_joint2 = self.j2_traj[_k + 1] - y2[-1]
        e_k_joint3 = self.j3_traj[_k + 1] - y3[-1]
        e_k_joint4 = self.j4_traj[_k + 1] - y4[-1]

        # Phi updates
        phi_k_joint1 = self.phi_estimator(phi_k_joint1, delta_u_1, delta_y_1, PHI_INIT=PHI_INIT1)
        phi_k_joint2 = self.phi_estimator(phi_k_joint2, delta_u_2, delta_y_2, PHI_INIT=PHI_INIT2)
        phi_k_joint3 = self.phi_estimator(phi_k_joint3, delta_u_3, delta_y_3, PHI_INIT=PHI_INIT3)
        phi_k_joint4 = self.phi_estimator(phi_k_joint4, delta_u_4, delta_y_4, PHI_INIT=PHI_INIT4)
        
        # Control action calculation
        u_k_joint_1 = self.control_action_calculator(phi_k_joint1, [u1[-1], 0, 0, 0], e_k_joint1)
        u1.append(u_k_joint_1)
        u_k_joint_2 = self.control_action_calculator(phi_k_joint2, [u2[-1], 0, 0, 0], e_k_joint2)
        u2.append(u_k_joint_2)
        u_k_joint_3 = self.control_action_calculator(phi_k_joint3, [u3[-1], 0, 0, 0], e_k_joint3)
        u3.append(u_k_joint_3)
        u_k_joint_4 = self.control_action_calculator(phi_k_joint4, [u4[-1], 0, 0, 0], e_k_joint4)
        u4.append(u_k_joint_4)

        # Target vector update
        next_tar[0] += u_k_joint_1
        next_tar[1] += u_k_joint_2
        next_tar[2] += u_k_joint_3
        next_tar[3] += u_k_joint_4

        self.update_target(next_tar)
        self.pub.publish(self.jointCmd)
        self.rate.sleep()
        # time.sleep(3.0/4.0*self.Ts) # Delay to match frequencies 

        _k = 2
        while _k < _N:
            # Delta u values 
            delta_u_1 = u1[-1] - u1[-2]
            delta_u_2 = u2[-1] - u2[-2]
            delta_u_3 = u3[-1] - u3[-2]
            delta_u_4 = u4[-1] - u4[-2]

            # Measured Positions
            y1.append(self.actual_positions[0])
            y2.append(self.actual_positions[1])
            y3.append(self.actual_positions[2])
            y4.append(self.actual_positions[3])

            # Delta y values
            delta_y_1 = y1[-1] - y1[-2]
            delta_y_2 = y2[-1] - y2[-2]
            delta_y_3 = y3[-1] - y3[-2]
            delta_y_4 = y4[-1] - y4[-2]
            
            # Error calculatiosn
            e_k_joint1 = self.j1_traj[_k + 1] - y1[-1]  
            e_k_joint2 = self.j2_traj[_k + 1] - y2[-1]  
            e_k_joint3 = self.j3_traj[_k + 1] - y3[-1]  
            e_k_joint4 = self.j4_traj[_k + 1] - y4[-1]  

            # Phi updates
            phi_k_joint1 = self.phi_estimator(phi_k_joint1, delta_u_1, delta_y_1, PHI_INIT=PHI_INIT1)
            phi_k_joint2 = self.phi_estimator(phi_k_joint2, delta_u_2, delta_y_2, PHI_INIT=PHI_INIT2)
            phi_k_joint3 = self.phi_estimator(phi_k_joint3, delta_u_3, delta_y_3, PHI_INIT=PHI_INIT3)
            phi_k_joint4 = self.phi_estimator(phi_k_joint4, delta_u_4, delta_y_4, PHI_INIT=PHI_INIT4)
            
            if _k >= 4:
                uvals_1 = [u1[-1], u1[-2], u1[-3], u1[-4]]
                uvals_2 = [u2[-1], u2[-2], u2[-3], u2[-4]]
                uvals_3 = [u3[-1], u3[-2], u3[-3], u3[-4]]
                uvals_4 = [u4[-1], u4[-2], u4[-3], u4[-4]]
            
            elif _k == 2:
                uvals_1 = [u1[-1], u1[-2], 0, 0]
                uvals_2 = [u2[-1], u2[-2], 0, 0]
                uvals_3 = [u3[-1], u3[-2], 0, 0]
                uvals_4 = [u4[-1], u4[-2], 0, 0]

            elif _k == 3:
                uvals_1 = [u1[-1], u1[-2], u1[-3], 0]
                uvals_2 = [u2[-1], u2[-2], u2[-3], 0]
                uvals_3 = [u3[-1], u3[-2], u3[-3], 0]
                uvals_4 = [u4[-1], u4[-2], u4[-3], 0]

            u_k_joint_1 = self.control_action_calculator(phi_k_joint1, uvals_1, e_k_joint1)
            u_k_joint_2 = self.control_action_calculator(phi_k_joint2, uvals_2, e_k_joint2)
            u_k_joint_3 = self.control_action_calculator(phi_k_joint3, uvals_3, e_k_joint3)
            u_k_joint_4 = self.control_action_calculator(phi_k_joint4, uvals_4, e_k_joint4)     
            u1.append(u_k_joint_1)
            u2.append(u_k_joint_2)
            u3.append(u_k_joint_3)
            u4.append(u_k_joint_4)           
            
            # Target vector update
            next_tar[0] += u_k_joint_1
            next_tar[1] += u_k_joint_2
            next_tar[2] += u_k_joint_3
            next_tar[3] += u_k_joint_4

            self.update_target(next_tar)
            self.pub.publish(self.jointCmd)
            self.rate.sleep()
            _k += 1
            # time.sleep(3.0/4.0*self.Ts)

        time_plt = range(0, int(_N))

        plt.subplot(2, 2, 1)
        plt.plot(time_plt, np.rad2deg(self.j1_traj[:-1]))
        plt.plot(time_plt, np.rad2deg(y1))

        plt.subplot(2, 2, 2)
        plt.plot(time_plt, np.rad2deg(self.j2_traj[:-1]))
        plt.plot(time_plt, np.rad2deg(y2))

        plt.subplot(2, 2, 3)
        plt.plot(time_plt, np.rad2deg(self.j3_traj[:-1]))
        plt.plot(time_plt, np.rad2deg(y3))

        plt.subplot(2, 2, 4)
        plt.plot(time_plt, np.rad2deg(self.j4_traj[:-1]))
        plt.plot(time_plt, np.rad2deg(y4))
        plt.show()

           

    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. The robot arm is moved to the home
        position first and then the joint(s) are updated randomly from there.
        """
        # rospy.Subscriber('/j2s6s200/joint_states', JointState, self.actual_values)  # Simulation
        rospy.Subscriber('/j2s6s200_driver/out/joint_state', JointState, self.actual_values)  # Real bot
        # self.move_joint_home()

        # print("******************************************************************")
        # print("\t\t\tNominal Motion")
        # print("******************************************************************")
        # self.nominal_trajectory()

        time.sleep(0.5)
        print("******************************************************************")
        print("\t\t\tAlgorithm Motion")
        print("******************************************************************")
        self.signal_update()

        # while not rospy.is_shutdown():
        #     # self.nominal_trajectory()
        #     self.signal_update()
        #     pass


if __name__ == '__main__':
    try:
        rospy.init_node('move_robot_using_trajectory_msg')
        # allow gazebo to launch
        time.sleep(1)

        # Unpause the physics
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # resp = unpause_gazebo()
        rt = RandomTrajectory([0.0, np.pi/2, np.pi, -3.0*np.pi/4.0, np.pi, 0.0], freq=10.0, runtime=120.0)
        rt.trajectory_calculator()
        rt.start()

    except rospy.ROSInterruptException:
        pass