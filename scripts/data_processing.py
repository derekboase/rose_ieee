import matplotlib.pyplot as plt
import pandas as pd


# filename = str(raw_input("Enter a filename:"))
filename = 'test'
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

print(Data)

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


for i in range(6, 9):
    plt.figure(2)
    plt.subplot(3, 1, i - 5)
    plt.plot(Data.index, Data.ix[:, i])
    plt.xlabel('Time, t (s)')
    plt.ylabel('Weight')
    plt.title('Joint 1 Gains vs. Time')
    plt.grid()
    plt.tight_layout()

for m in range(9, 12):
    plt.figure(3)
    plt.subplot(3, 1, m - 8)
    plt.plot(Data.index, Data.ix[:, m])
    plt.xlabel('Time, t (s)')
    plt.ylabel('Weight')
    plt.title('Joint 2 Gains vs. Time')
    plt.grid()
    plt.tight_layout()

for n in range(12, 15):
    plt.figure(4)
    plt.subplot(3, 1, n - 11)
    plt.plot(Data.index, Data.ix[:, n])
    plt.xlabel('Time, t (s)')
    plt.ylabel('Weight')
    plt.title('Joint 3 Gains vs. Time')
    plt.grid()
    plt.tight_layout()

plt.show()




