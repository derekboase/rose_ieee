#! /usr/bin/env python

"""Randomly chooses a robot trajectory in joint space by randomly updating the current pose"""

import matplotlib.pyplot as plt
import numpy as np
import rospy
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
# from std_srvs.srv import Empty
# from random import uniform


def generate_SPD_matrix(n):
    """
    Returns
    :param n: Number of dimensions for the symmetric positive definite matrix
    :return: np array of dimensions nxn
    """
    A = np.random.rand(n, n)
    return 1/2.0 * np.matmul(A, np.transpose(A))


def bound(low, high, val):
    return np.array([max(low, min(high, val))])


class RandomTrajectory:
    def __init__(self, home_joints, freq=10.0, runtime=10.0):
        # All the setup stuff for the nodes and topics
        self.topic_name = '/j2s6s200/effort_joint_trajectory_controller/command'
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

        self.actual_positions = np.zeros((1, 6))
        self.actual_velocities = np.zeros((1, 6))

        self.idx = 0

        self.algorithm = []

    def trajectory_calculator(self):
        """
        This function calculates the joint space positions of the nominal trajectory given an equation
        :return: None
        """
        # Trajectory points
        _time = np.arange(0, self.end_time + self.Ts, step=self.Ts)
        _tau = 3
        _amplitude = np.pi
        for t in _time:
            if t <= self.end_time*4/5:
                # self.j1_traj.append(_amplitude * (1 - np.exp(-_tau * t / np.pi)))
                self.j1_traj.append(np.pi/4)
            else:
                # self.j1_traj.append(_amplitude * np.exp(-_tau * (t - self.end_time/2) / np.pi))
                self.j1_traj.append(0)

    def actual_values(self, real_joint_angles):
        self.actual_positions = np.array(real_joint_angles.position[0:6])
        self.actual_velocities = np.array(real_joint_angles.velocity[0:6])

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

    def nominal_trajectory(self):
        for self.idx, j1 in enumerate(self.j1_traj):
            next_tar = np.array(self.current_joints)
            next_tar[0] = j1
            self.update_target(next_tar)
            # print(self.point)
            # while True:
            #     pass
            # print(self.jointCmd)
            self.pub.publish(self.jointCmd)
            self.rate.sleep()
            self.idx += 1

    def update_target(self, next_targets):
        self.jointCmd.points = []
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(self.Ts)
        # self.point.velocities = ((next_targets - np.array(self.current_joints)) / self.Ts).tolist()
        # self.point.accelerations = (np.array(self.point.velocities) / self.Ts).tolist()
        # self.current_joints = next_targets.tolist()
        # self.point.positions = self.current_joints
        self.point.positions = next_targets.tolist()
        self.jointCmd.points.append(self.point)

    def signal_update(self):
        # Initialization of algorithm constants
        _N = self.end_time / self.Ts
        _zeta_actor = 0.01  # Choose the actor to adapt more slowly than the critic
        _zeta_critic = 0.05
        Q = generate_SPD_matrix(3)
        R = generate_SPD_matrix(1)
        delta_conv, window_conv = 1e-2, _N/10

        _k, _weights_conv = 0, False  # Index and convergence flag

        _E_k = np.zeros((3, 1))  # shape(3, 1)
        _E_k1 = np.zeros((3, 1))  # shape(3, 1)

        _Wc_1 = generate_SPD_matrix(4)  # shape(4, 4)
        _Wa_1 = (-1/_Wc_1[3][3]*_Wc_1[3][0:3]).reshape(1, 3)  # shape(1, 3)
        # _Wa_1 = np.zeros((1, 3))  # shape(1, 3)

        next_tar = np.array(self.current_joints)  # load the next target variable with the current joints
        while _k < _N and not _weights_conv:
            # Calculate the control signal: Step 6 for Joint 1 Position
            _u_hat = bound(-0.35, 0.35, float(np.matmul(_Wa_1, _E_k)))  # shape(1,)
            next_tar[0] += _u_hat  # shape(1,)
            self.update_target(next_tar)
            self.algorithm.append(self.actual_positions[0])  # For the sole purpose of
            self.pub.publish(self.jointCmd)  # Publish the
            self.rate.sleep()

            # Find V and U: Step 8 for Joint 1 Position
            _Eu_concat = np.concatenate((_E_k, _u_hat.reshape(-1, 1)), axis=0)
            _Eu_transpose = np.transpose(_Eu_concat)
            _V_k = 1/2.0 * np.matmul(np.matmul(_Eu_transpose, _Wc_1), _Eu_concat)
            _E_transpose = np.transpose(_E_k)
            _U_k = 1/2.0 * (np.matmul(np.matmul(_E_transpose, Q), _E_k) + _u_hat*R*_u_hat)

            # Get E(k + 1), u_hat and V(k_1): Step 9 for Joint 1 Position
            _E_k1[2] = _E_k[1]
            _E_k1[1] = _E_k[0]
            _E_k1[0] = self.j1_traj[_k + 1] - self.actual_positions[0]
            _u_hat_k1 = bound(-0.35, 0.35, float(np.matmul(_Wa_1, _E_k1)))  # shape(1,)
            _Z = np.concatenate((_E_k1, _u_hat_k1.reshape(-1, 1)), axis=0)
            _Z_trans = np.transpose(_Z)
            _V_k1 = 1/2.0*np.matmul(np.matmul(_Z_trans, _Wc_1), _Z)

            # Update critic weights: Step 11 for Joint 1 Position
            temp = _zeta_critic*(_V_k - (_U_k + _V_k1))
            # temp = _zeta_critic*(_V_k - (_U_k + _V_k1))
            _Wc_1 -= temp*np.matmul(_Z, _Z_trans)

            # Update actor weights: Step 12 for Joint 1 Position
            _Wa_1 -= _zeta_actor*(np.matmul(_Wa_1, _E_k) - (-1/_Wc_1[3][3]*np.matmul(_Wc_1[3][0:3], _E_k)))*_E_transpose

            # Updates
            _E_k = _E_k1
            _k += 1

        return _Wc_1, _Wa_1

    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. The robot arm is moved to the home
        position first and then the joint(s) are updated randomly from there.
        """
        rospy.Subscriber('/j2s6s200/joint_states', JointState, self.actual_values)
        self.move_joint_home()
        # print("******************************************************************")
        # print("\t\t\tNominal Motion")
        # print("******************************************************************")
        # self.nominal_trajectory()
        print("******************************************************************")
        print("\t\t\tAlgorithm Motion")
        print("******************************************************************")
        self.signal_update()
        t = np.arange(0, self.end_time + self.Ts, step=self.Ts)
        plt.plot(2*t, self.j1_traj)
        plt.plot(2*t[1:], self.algorithm)
        plt.legend(['nominal', 'actual'])
        plt.show()

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

        rt = RandomTrajectory([0.0, 2.9, 1.3, -2.1, 1.4, 0.0], freq=2.0, runtime=30.0)
        # rt = RandomTrajectory([0.0, 2.0, 1.3, -2.1, 1.4, 0.0], freq=2.0, runtime=60.0)
        # rt = RandomTrajectory(np.deg2rad([180, 270, 90, 270, 270, 270]))
        rt.trajectory_calculator()
        rt.start()

    except rospy.ROSInterruptException:
        pass
