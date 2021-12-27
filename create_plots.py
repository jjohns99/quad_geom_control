import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from so3_py import Quat2Euler

def main():
    # data = np.load("data/fast_circles.npy")
    # t = data[:,0]
    # px = data[:,1]
    # py = data[:,2]
    # pz = data[:,3]
    # vx = data[:,4]
    # vy = data[:,5]
    # vz = data[:,6]

    # euler = np.zeros((data.shape[0], 3))
    # for i in range(data.shape[0]):
    #   euler[i,:] = np.degrees(Quat2Euler(data[i,7:11]).T)

    # phi = euler[:,0]
    # theta = euler[:,1]
    # psi = euler[:,2]

    # px_des = data[:,1+13]
    # py_des = data[:,2+13]
    # pz_des = data[:,3+13]
    # vx_des = data[:,4+13]
    # vy_des = data[:,5+13]
    # vz_des = data[:,6+13]

    # euler_des = np.zeros((data.shape[0], 3))
    # for i in range(data.shape[0]):
    #   euler_des[i,:] = np.degrees(Quat2Euler(data[i,7+13:11+13]).T)

    # phi_des = euler_des[:,0]
    # theta_des = euler_des[:,1]
    # psi_des = euler_des[:,2]

    # fig, ax = plt.subplots(6, 1)
    # ax[0].set_title("Fast Circles Trajectory")

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
    # ax[5].legend(["actual", "desired"], loc="lower right")

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
    # ax.set_title("Fast Circles Position")
    # ax.legend(["actual", "desired"], loc='center right')
    # # ax.set_xlim([-1, 1])
    # # ax.set_xticks(np.linspace(-1.0, 1.0, 5))

    data_ours = np.load("data/upside_down_ours.npy")
    t_ours = data_ours[:,0]
    px_ours = data_ours[:,1]
    py_ours = data_ours[:,2]
    pz_ours = data_ours[:,3]
    vx_ours = data_ours[:,4]
    vy_ours = data_ours[:,5]
    vz_ours = data_ours[:,6]

    euler_ours = np.zeros((data_ours.shape[0], 3))
    for i in range(data_ours.shape[0]):
      euler_ours[i,:] = np.degrees(Quat2Euler(data_ours[i,7:11]).T)

    phi_ours = euler_ours[:,0]
    theta_ours = euler_ours[:,1]
    psi_ours = euler_ours[:,2]

    data_lee1 = np.load("data/upside_down_lee1.npy")
    t_lee1 = data_lee1[:,0]
    px_lee1 = data_lee1[:,1]
    py_lee1 = data_lee1[:,2]
    pz_lee1 = data_lee1[:,3]
    vx_lee1 = data_lee1[:,4]
    vy_lee1 = data_lee1[:,5]
    vz_lee1 = data_lee1[:,6]

    euler_lee1 = np.zeros((data_lee1.shape[0], 3))
    for i in range(data_lee1.shape[0]):
      euler_lee1[i,:] = np.degrees(Quat2Euler(data_lee1[i,7:11]).T)

    phi_lee1 = euler_lee1[:,0]
    theta_lee1 = euler_lee1[:,1]
    psi_lee1 = euler_lee1[:,2]

    data_lee3 = np.load("data/upside_down_lee3.npy")
    t_lee3 = data_lee3[:,0]
    px_lee3 = data_lee3[:,1]
    py_lee3 = data_lee3[:,2]
    pz_lee3 = data_lee3[:,3]
    vx_lee3 = data_lee3[:,4]
    vy_lee3 = data_lee3[:,5]
    vz_lee3 = data_lee3[:,6]

    euler_lee3 = np.zeros((data_lee3.shape[0], 3))
    for i in range(data_lee3.shape[0]):
      euler_lee3[i,:] = np.degrees(Quat2Euler(data_lee3[i,7:11]).T)

    phi_lee3 = euler_lee3[:,0]
    theta_lee3 = euler_lee3[:,1]
    psi_lee3 = euler_lee3[:,2]

 
    fig, ax = plt.subplots(6, 1)
    ax[0].set_title("Upside-down Recovery")

    ax[0].plot(t_ours, px_ours)
    ax[0].plot(t_lee1, px_lee1, 'g')
    ax[0].plot(t_lee3, px_lee3, 'r')
    ax[0].set_ylabel("px (m)", fontsize=8)
    ax[0].set_ylim([-1, 1])
    ax[1].plot(t_ours, py_ours)
    ax[1].plot(t_lee1, py_lee1, 'g')
    ax[1].plot(t_lee3, py_lee3, 'r')
    ax[1].set_ylabel("py (m)", fontsize=8)
    ax[1].set_ylim([-7, 3])
    ax[2].plot(t_ours, pz_ours)
    ax[2].plot(t_lee1, pz_lee1, 'g')
    ax[2].plot(t_lee3, pz_lee3, 'r')
    ax[2].set_ylabel("pz (m)", fontsize=8)
    ax[2].set_ylim([-3, 5])

    ax[3].plot(t_ours, phi_ours)
    ax[3].plot(t_lee1, phi_lee1, 'g')
    ax[3].plot(t_lee1, phi_lee3, 'r')
    ax[3].set_ylabel("roll (deg)", fontsize=8)
    ax[4].plot(t_ours, theta_ours)
    ax[4].plot(t_lee1, theta_lee1, 'g')
    ax[4].plot(t_lee1, theta_lee3, 'r')
    ax[4].set_ylabel("pitch (deg)", fontsize=8)
    ax[5].plot(t_ours, psi_ours)
    ax[5].plot(t_lee1, psi_lee1, 'g')
    ax[5].plot(t_lee3, psi_lee3, 'r')
    ax[5].set_ylabel("yaw (deg)", fontsize=8)
    ax[5].set_xlabel("time (seconds)")
    ax[5].legend(["ours", "frobenius", "hybrid"], loc='lower right')

    plt.show()

if __name__=="__main__":
    main()