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
        self.topic_name = '/j2s6s200/effort_joint_trajectory_controller/command'  # Simulation
        # self.topic_name = '/j2s6s200_driver/trajectory_controller/command'  # Real bot
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

        self.actual_positions = np.zeros((1, 6))
        self.actual_velocities = np.zeros((1, 6))

        self.idx = 0

        self.algorithm_j1 = []
        self.algorithm_j2 = []

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
            if t <= self.end_time/2:
                self.j1_traj.append(_amplitude * (1 - np.exp(-t/_tau)))

            else:
                self.j1_traj.append(_amplitude * np.exp(-(t - self.end_time/2)/_tau))
                # self.j1_traj.append(0)
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

    def nominal_trajectory(self):
        for j1, j2 in zip(self.j1_traj, self.j2_traj):
            next_tar = np.array(self.current_joints)
            next_tar[0] = j1
            next_tar[1] = j2
            self.update_target(next_tar)
            # print(self.point)
            # while True:
            #     pass
            # print(self.jointCmd)
            self.pub.publish(self.jointCmd)
            self.rate.sleep()

    def update_target(self, next_targets):
        self.jointCmd.points = []
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(self.Ts)
        self.point.velocities = ((next_targets - np.array(self.current_joints)) / self.Ts).tolist()
        # self.point.accelerations = (np.array(self.point.velocities) / self.Ts).tolist()
        self.current_joints = next_targets.tolist()
        # self.point.positions = self.current_joints
        self.point.positions = next_targets.tolist()
        self.jointCmd.points.append(self.point)

    def signal_update(self):
        global _Wa_1_1, _Wa_1_2, _Wa_1_3, _k_lst
        global _Wa_2_1, _Wa_2_2, _Wa_2_3

        _Wa_1_1, _Wa_1_2, _Wa_1_3, _k_lst = [], [], [], []
        _Wa_2_1, _Wa_2_2, _Wa_2_3 = [], [], []


        # Initialization of algorithm constants
        _N = self.end_time / self.Ts
        _zeta_actor = 0.01  # Choose the actor to adapt more slowly than the critic
        _zeta_critic = 0.05
        # _Q_1 = generate_SPD_matrix(3)
        # _R_1 = generate_SPD_matrix(1)
        # _Q_2 = generate_SPD_matrix(3)
        # _R_2 = generate_SPD_matrix(1)

        _Q_1 = np.array([[0.51502986, 0.25789362, 0.06580822],
                         [0.25789362, 0.19214249, 0.0747135],
                         [0.06580822, 0.0747135, 0.0378436]])
        _R_1 = np.array([[0.07450969]])

        _Q_2 = np.array([[0.8503787, 0.59430608, 0.4799643],
                          [0.50802145, 0.51991574, 0.2456848],
                          [0.4799643, 0.2456848, 0.38590322]])
        _R_2 = np.array([[0.00876479]])

        lam1, vec1 = np.linalg.eig(_Q_1)
        lam2, vec2 = np.linalg.eig(_Q_2)

        delta_conv, window_conv = 1e-2, _N/10

        _k, _weights_conv = 0, False  # Index and convergence flag

        _E_k_1 = np.zeros((3, 1))  # shape(3, 1)
        _E_k1_1 = np.zeros((3, 1))  # shape(3, 1)

        _E_k_2 = np.zeros((3, 1))  # shape(3, 1)
        _E_k1_2 = np.zeros((3, 1))  # shape(3, 1)

        # _Wc_1 = generate_SPD_matrix(4)  # shape(4, 4)
        # _Wc_2 = generate_SPD_matrix(4)

        _Wc_1 = np.array([[ 0.80349833,  0.30936819,  0.84494049,  0.71454207],
                          [ 0.30936819,  0.21330422,  0.31156708,  0.36979277],
                          [ 0.84494049,  0.31156708,  1.09927468,  0.53434843],
                          [ 0.71454207,  0.36979277,  0.53434843,  0.9285541 ]])

        _Wc_2 = np.array([[ 0.42471294, 0.49183218, 0.54214396, 0.52932378],
                          [ 0.49183218, 0.66047158, 0.78537323, 0.5942725 ],
                          [ 0.54214396,  0.78537323, 1.15277585, 0.89469537],
                          [ 0.52932378, 0.5942725,  0.89469537, 1.05989052]])

        noise = 0.15
        _Wa_1 = (1/_Wc_1[3][3]*_Wc_1[3][0:3]).reshape(1, 3)  # shape(1, 3) NEGATED
        # _Wa_1[0, 0] = _Wa_1[0, 0] + np.random.normal(scale=noise)
        # _Wa_1[0, 1] = _Wa_1[0, 1] + np.random.normal(scale=noise)
        # _Wa_1[0, 2] = _Wa_1[0, 2] + np.random.normal(scale=noise)

        _Wa_2 = (1/_Wc_2[3][3]*_Wc_2[3][0:3]).reshape(1, 3)  # shape(1, 3) NEGATED
        # _Wa_2[0, 0] = _Wa_2[0, 0] + np.random.normal(scale=noise)
        # _Wa_2[0, 1] = _Wa_2[0, 1] + np.random.normal(scale=noise)
        # _Wa_2[0, 2] = _Wa_2[0, 2] + np.random.normal(scale=noise)

        _Wa_1[0, 1] *= -1
        _Wa_2[0, 1] *= -1

        print('''Wc_1 is:
{0} 
The matrix Q1 is:
{1}
The matrix R1 is:
{2}
The eigenvalues of matrix Q1 are:
{3}'''.format(_Wc_1, _Q_1, _R_1, lam1))

        print('''Wc_2 is:
{0} 
The matrix Q2 is:
{1}
The matrix R2 is:
{2}
The eigenvalues of matrix Q2 are:
{3}'''.format(_Wc_2, _Q_2, _R_2, lam2))

        next_tar = np.array(self.current_joints)  # load the next target variable with the current joints

        while _k < _N and not _weights_conv:
            _Wa_1_1.append(_Wa_1[0, 0])
            _Wa_1_2.append(_Wa_1[0, 1])
            _Wa_1_3.append(_Wa_1[0, 2])

            _Wa_2_1.append(_Wa_2[0, 0])
            _Wa_2_2.append(_Wa_2[0, 1])
            _Wa_2_3.append(_Wa_2[0, 2])

            _k_lst.append(_k)
            # Calculate the control signal: Step 6 for Joint 1 Position
            _u_hat_1 = bound(-0.35, 0.35, float(np.matmul(_Wa_1, _E_k_1)))  # shape(1,)
            _u_hat_2 = bound(-0.35, 0.35, float(np.matmul(_Wa_2, _E_k_2)))  # shape(1,)

            next_tar[0] += _u_hat_1  # shape(1,)
            next_tar[1] += _u_hat_2  # shape(1,)

            self.update_target(next_tar)
            self.algorithm_j1.append(self.actual_positions[0])  # For the sole purpose of plotting
            self.algorithm_j2.append(self.actual_positions[1])  # For the sole purpose of plotting
            self.pub.publish(self.jointCmd)  # Publish the
            self.rate.sleep()

            # Find V and U: Step 8 for Joint 1 Position
            _Eu_concat_1 = np.concatenate((_E_k_1, _u_hat_1.reshape(-1, 1)), axis=0)
            _Eu_transpose_1 = np.transpose(_Eu_concat_1)
            _V_k_1 = 1/2.0 * np.matmul(np.matmul(_Eu_transpose_1, _Wc_1), _Eu_concat_1)
            _E_transpose_1 = np.transpose(_E_k_1)
            _eqe_1 = np.matmul(np.matmul(_E_transpose_1, _Q_1), _E_k_1)
            _uru_1 = _u_hat_1*_R_1*_u_hat_1
            _U_k_1 = 1 / 2.0 * (_eqe_1 + _uru_1)
            print('For k={0}\nu_hat1={1}\nWa1={2}'.format(_k, _u_hat_1, _Wa_1))

            # Find V and U: Step 8 for Joint 2 Position
            _Eu_concat_2 = np.concatenate((_E_k_2, _u_hat_2.reshape(-1, 1)), axis=0)
            _Eu_transpose_2 = np.transpose(_Eu_concat_2)
            _V_k_2 = 1/2.0 * np.matmul(np.matmul(_Eu_transpose_2, _Wc_2), _Eu_concat_2)
            _E_transpose_2 = np.transpose(_E_k_2)
            _eqe_2 = np.matmul(np.matmul(_E_transpose_2, _Q_2), _E_k_2)
            _uru_2 = _u_hat_2*_R_2*_u_hat_2
            _U_k_2 = 1 / 2.0 * (_eqe_2 + _uru_2)
            print('For k={0}\nu_hat2={1}\nWa2={2}'.format(_k, _u_hat_2, _Wa_2))

            # Get E(k + 1), u_hat and V(k_1): Step 9 for Joint 1 Position
            _E_k1_1[2] = _E_k_1[1]
            _E_k1_1[1] = _E_k_1[0]
            _E_k1_1[0] = self.j1_traj[_k + 1] - self.actual_positions[0]

            # Get E(k + 1), u_hat and V(k_1): Step 9 for Joint 2 Position
            _E_k1_2[2] = _E_k_2[1]
            _E_k1_2[1] = _E_k_2[0]
            _E_k1_2[0] = self.j2_traj[_k + 1] - self.actual_positions[1]

            _u_hat_k1_1 = bound(-0.35, 0.35, float(np.matmul(_Wa_1, _E_k1_1)))  # shape(1,)
            _Z_1 = np.concatenate((_E_k1_1, _u_hat_k1_1.reshape(-1, 1)), axis=0)
            _Z_trans_1 = np.transpose(_Z_1)
            _V_k1_1 = 1/2.0*np.matmul(np.matmul(_Z_trans_1, _Wc_1), _Z_1)

            _u_hat_k1_2 = bound(-0.35, 0.35, float(np.matmul(_Wa_2, _E_k1_2)))  # shape(1,)
            _Z_2 = np.concatenate((_E_k1_2, _u_hat_k1_2.reshape(-1, 1)), axis=0)
            _Z_trans_2 = np.transpose(_Z_2)
            _V_k1_2 = 1 / 2.0 * np.matmul(np.matmul(_Z_trans_2, _Wc_2), _Z_2)

            # Update critic weights: Step 11 for Joint 1 Position
            temp = _zeta_critic*(_V_k_1 - (_U_k_1 + _V_k1_1))
            _Wc_1 -= temp*np.matmul(_Z_1, _Z_trans_1)
            _Wa_1 -= _zeta_actor*(np.matmul(_Wa_1, _E_k_1) -
                                  (-1/_Wc_1[3][3]*np.matmul(_Wc_1[3][0:3], _E_k_1)))*_E_transpose_1

            # Update critic weights: Step 11 for Joint 1 Position
            temp = _zeta_critic*(_V_k_2 - (_U_k_2 + _V_k1_2))
            _Wc_2 -= temp*np.matmul(_Z_2, _Z_trans_2)
            _Wa_2 -= _zeta_actor*(np.matmul(_Wa_2, _E_k_2) -
                                  (-1/_Wc_2[3][3]*np.matmul(_Wc_2[3][0:3], _E_k_2)))*_E_transpose_2

            # Updates
            _E_k_1 = _E_k1_1
            _E_k_2 = _E_k1_2
            _k += 1

        return _Wc_1, _Wa_1

    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. The robot arm is moved to the home
        position first and then the joint(s) are updated randomly from there.
        """
        rospy.Subscriber('/j2s6s200/joint_states', JointState, self.actual_values)  # Simulation
        # rospy.Subscriber('/j2s6s200_driver/out/joint_states', JointState, self.actual_values)  # Real bot
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

        plt.figure(1)
        plt.subplot(2, 1, 1)
        plt.plot(2*t, np.rad2deg(self.j1_traj))
        plt.plot(2*t[1:], np.rad2deg(self.algorithm_j1))
        plt.legend(['nominal', 'actual'])

        plt.subplot(2, 1, 2)
        plt.plot(2*t, np.rad2deg(self.j2_traj))
        plt.plot(2*t[1:], np.rad2deg(self.algorithm_j2))
        plt.legend(['nominal', 'actual'])
        plt.show()

        plt.figure(2)
        plt.subplot(2, 1, 1)
        plt.plot(_k_lst, np.array(_Wa_1_1))
        # plt.plot(_k_lst, np.array(_Wa_1_2))
        # plt.plot(_k_lst, np.array(_Wa_1_3))
        plt.legend(['Wa1_1', 'Wa1_2', 'Wa1_3'])

        plt.subplot(2, 1, 2)
        plt.plot(_k_lst[0:20], np.array(_Wa_2_1[0:20]))
        # plt.plot(_k_lst, np.array(_Wa_2_2))
        # plt.plot(_k_lst, np.array(_Wa_2_3))
        plt.legend(['Wa2_1', 'Wa2_2', 'Wa2_3'])
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

        rt = RandomTrajectory([0.0, np.pi/2, np.pi, -2.1, np.pi, 0.0], freq=2.0, runtime=60.0)
        # rt = RandomTrajectory([0.0, 2.0, 1.3, -2.1, 1.4, 0.0], freq=2.0, runtime=60.0)
        # rt = RandomTrajectory(np.deg2rad([180, 270, 90, 270, 270, 270]))
        rt.trajectory_calculator()
        rt.start()

    except rospy.ROSInterruptException:
        pass
