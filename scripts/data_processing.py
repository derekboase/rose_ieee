import matplotlib.pyplot as plt
import pandas as pd


# for filename in ['neg_111', 'neg_112', 'neg_212', 'neg_211']:
for filename in ['interesting_10Hz']:
    folder = 'distributed_weight_all_joints'
    save_folder = '10Hz'
    # folder = 'distributed_weight_all_joints'
    # filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/' + folder + '/' + filename + '.csv'
    filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/' + filename + '.csv'
    col_headers = ['time',
                   'Joint 1 Nominal',
                   'Joint 2 Nominal',
                   'Joint 3 Nominal',
                   'Joint 1 Algorithm',
                   'Joint 2 Algorithm',
                   'Joint 3 Algorithm',
                   'Joint 1 Actor 1',
                   'Joint 1 Actor 2',
                   'Joint 1 Actor 3',
                   'Joint 2 Actor 1',
                   'Joint 2 Actor 2',
                   'Joint 2 Actor 3',
                   'Joint 3 Actor 1',
                   'Joint 3 Actor 2',
                   'Joint 3 Actor 3']

    Data = pd.read_csv(filepath, header=None, names=col_headers, index_col='time')

    save_filepath = filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/pictures/' + save_folder + '/' \
                               + filename + '_traj.png'
    print(save_filepath)
    for idx in range(1, 4):
        plt.figure(1)
        plt.subplot(3, 1, idx)
        plt.plot(Data.index, Data.ix[:, idx - 1])
        plt.plot(Data.index, Data.ix[:, idx + 2])
        plt.xlabel('Time, t (s)')
        plt.ylabel('Angle, deg.')
        plt.title('Nominal Trajectory and Algorithmic Trajectory vs. Time: Joint {}'.format(idx))
        plt.grid()
        plt.tight_layout()
    plt.savefig(save_filepath, format='png')

    save_filepath = filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/pictures/' + save_folder + '/' \
                               + filename + '_j1_weights.png'
    for i in range(6, 9):
        plt.figure(2)
        plt.plot(Data.index, Data.ix[:, i])
        plt.xlabel('Time, t (s)')
        plt.ylabel('Weight')
        plt.title('Joint 1 Gains vs. Time')
        plt.legend(['K', '$K_\Delta$', '$K_{2\Delta}$'])
        plt.grid()
        plt.tight_layout()

    plt.savefig(save_filepath, format='png')

    save_filepath = filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/pictures/' + save_folder + '/' \
                               + filename + '_j2_weights.png'
    for m in range(9, 12):
        plt.figure(3)
        plt.plot(Data.index, Data.ix[:, m])
        plt.xlabel('Time, t (s)')
        plt.ylabel('Weight')
        plt.title('Joint 2 Gains vs. Time')
        plt.legend(['K', '$K_\Delta$', '$K_{2\Delta}$'])
        plt.grid()
        plt.tight_layout()

    plt.savefig(save_filepath, format='png')

    save_filepath = filepath = '/home/derek/catkin_ws/src/rose_ieee/scripts/Data/pictures/' + save_folder + '/' \
                               + filename + '_j3_weights.png'
    for n in range(12, 15):
        plt.figure(4)
        plt.plot(Data.index, Data.ix[:, n])
        plt.xlabel('Time, t (s)')
        plt.ylabel('Weight')
        plt.title('Joint 3 Gains vs. Time')
        plt.legend(['K', '$K_\Delta$', '$K_{2\Delta}$'])
        plt.grid()
        plt.tight_layout()

    plt.savefig(save_filepath, format='png')

    plt.close('all')

    print(Data)
