import numpy as np

# read from sim_A1M.npy
sim_A1M = np.load("sim_A1M.npy")
print("sim_A1M:\n", sim_A1M)

est_A1M = np.load("est_A1M.npy")
print("est_A1M:\n", est_A1M)

print("\nMSE:\n", np.mean((sim_A1M - est_A1M) ** 2))
