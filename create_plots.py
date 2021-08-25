import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from so3 import Quat2Euler

def main():
    data = np.load("data/upside_down_log.npy")
    t = data[:,0]
    px = data[:,1]
    py = data[:,2]
    pz = data[:,3]
    vx = data[:,4]
    vy = data[:,5]
    vz = data[:,6]

    euler = np.zeros((data.shape[0], 3))
    for i in range(data.shape[0]):
      euler[i,:] = np.degrees(Quat2Euler(data[i,7:11]).T)

    phi = euler[:,0]
    theta = euler[:,1]
    psi = euler[:,2]

    px_des = data[:,1+13]
    py_des = data[:,2+13]
    pz_des = data[:,3+13]
    vx_des = data[:,4+13]
    vy_des = data[:,5+13]
    vz_des = data[:,6+13]

    euler_des = np.zeros((data.shape[0], 3))
    for i in range(data.shape[0]):
      euler_des[i,:] = np.degrees(Quat2Euler(data[i,7+13:11+13]).T)

    phi_des = euler_des[:,0]
    theta_des = euler_des[:,1]
    psi_des = euler_des[:,2]

    # fig, ax = plt.subplots(6, 1)
    # ax[0].set_title("Flipping Loops Trajectory")

    # ax[0].plot(t, px)
    # ax[0].plot(t, px_des)
    # ax[0].set_ylabel("px (m)", fontsize=8)
    # ax[1].plot(t, py)
    # ax[1].plot(t, py_des)
    # ax[1].set_ylabel("py (m)", fontsize=8)
    # ax[2].plot(t, pz)
    # ax[2].plot(t, pz_des)
    # ax[2].set_ylabel("pz (m)", fontsize=8)

    # ax[3].plot(t, phi)
    # ax[3].plot(t, phi_des)
    # ax[3].set_ylabel("roll (deg)", fontsize=8)
    # ax[4].plot(t, theta)
    # ax[4].plot(t, theta_des)
    # ax[4].set_ylabel("pitch (deg)", fontsize=8)
    # ax[5].plot(t, psi)
    # ax[5].plot(t, psi_des)
    # ax[5].set_ylabel("yaw (deg)", fontsize=8)
    # ax[5].set_xlabel("time (seconds)")
    # ax[5].legend(["actual", "desired"])

    # # fig.tight_layout()

    # plt.figure()

    # ax = plt.axes(projection='3d')
    # plt.rcParams['axes.titley'] = 1.0 
    # plt.rcParams['axes.titlepad'] = -14 

    # ax.plot3D(px, py, -pz)
    # ax.plot3D(px_des, py_des, -pz_des)
    # ax.set_xlabel("x (m)")
    # ax.set_ylabel("y (m)")
    # ax.set_zlabel("z (m)")
    # ax.set_title("Flipping Loops Position")
    # ax.legend(["actual", "desired"], loc='center right')
    # ax.set_xlim([-1, 1])

    data_tr = np.load("data/upside_down_trace.npy")
    t_tr = data_tr[:,0]
    px_tr = data_tr[:,1]
    py_tr = data_tr[:,2]
    pz_tr = data_tr[:,3]
    vx_tr = data_tr[:,4]
    vy_tr = data_tr[:,5]
    vz_tr = data_tr[:,6]

    euler_tr = np.zeros((data_tr.shape[0], 3))
    for i in range(data_tr.shape[0]):
      euler_tr[i,:] = np.degrees(Quat2Euler(data_tr[i,7:11]).T)

    phi_tr = euler_tr[:,0]
    theta_tr = euler_tr[:,1]
    psi_tr = euler_tr[:,2]

    px_des_tr = data_tr[:,1+13]
    py_des_tr = data_tr[:,2+13]
    pz_des_tr = data_tr[:,3+13]
    vx_des_tr = data_tr[:,4+13]
    vy_des_tr = data_tr[:,5+13]
    vz_des_tr = data_tr[:,6+13]

    euler_tr_des = np.zeros((data_tr.shape[0], 3))
    for i in range(data_tr.shape[0]):
      euler_tr_des[i,:] = np.degrees(Quat2Euler(data_tr[i,7+13:11+13]).T)

    phi_des_tr = euler_tr_des[:,0]
    theta_des_tr = euler_tr_des[:,1]
    psi_des_tr = euler_tr_des[:,2]

 
    fig, ax = plt.subplots(6, 1)
    ax[0].set_title("Upside-down Recovery")

    ax[0].plot(t, px)
    ax[0].plot(t_tr, px_tr)
    ax[0].plot(t, px_des)
    ax[0].set_ylabel("px (m)", fontsize=8)
    ax[0].set_ylim([-1, 1])
    ax[1].plot(t, py)
    ax[1].plot(t_tr, py_tr)
    ax[1].plot(t, py_des)
    ax[1].set_ylabel("py (m)", fontsize=8)
    ax[1].set_ylim([-3, 3])
    ax[2].plot(t, pz)
    ax[2].plot(t_tr, pz_tr)
    ax[2].plot(t, pz_des)
    ax[2].set_ylabel("pz (m)", fontsize=8)
    ax[2].set_ylim([-3, 10])

    ax[3].plot(t, phi)
    ax[3].plot(t_tr, phi_tr)
    ax[3].set_ylabel("roll (deg)", fontsize=8)
    ax[4].plot(t, theta)
    ax[4].plot(t_tr, theta_tr)
    ax[4].set_ylabel("pitch (deg)", fontsize=8)
    ax[5].plot(t, psi)
    ax[5].plot(t_tr, psi_tr)
    ax[5].set_ylabel("yaw (deg)", fontsize=8)
    ax[5].set_xlabel("time (seconds)")
    ax[5].legend(["log error", "trace error"], loc='lower right')

    plt.show()

if __name__=="__main__":
    main()