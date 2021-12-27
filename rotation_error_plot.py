import numpy as np
from so3 import *
import matplotlib.pyplot as plt

def EulerPhi2Rot(phi):
  e = np.array([phi, 0.0, 0.0])
  return Quat2Rotation(Euler2Quat(e))

def norm_trace_err(R):
  return np.linalg.norm(vee(skew_sym(R)))

def norm_wierd_trace_err(R):
  return np.linalg.norm(vee(skew_sym(R))/(np.sqrt(1+np.trace(R))))

def norm_log_err(R):
  return np.linalg.norm(Log(R))


def main():
  phi = np.linspace(0.0, np.pi, 1000)
  rot = [Exp(np.array([p,0.0,0.0])) for p in phi]
  phi_tr = [norm_trace_err(R) for R in rot]
  phi_wierd_tr = [norm_wierd_trace_err(R) for R in rot]
  phi_log = [norm_log_err(R) for R in rot]

  plt.plot(phi, phi_tr)
  # plt.plot(phi, phi_wierd_tr)
  # plt.plot(phi[:-1], phi_log[:-1])
  plt.title("Error Magnitude Versus Rotation")

  plt.show()


if __name__=="__main__":
  main()