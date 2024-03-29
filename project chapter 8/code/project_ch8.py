import numpy as np
import modern_robotics as mr
import pandas as pd
np.set_printoptions(suppress=True, precision=3)

# UR5 Robot Parameters

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]


# Conditions for the simulation

g = np.array([0, 0, -9.81])
taulist = [0, 0, 0, 0, 0, 0]
dthetalist = [0, 0, 0, 0, 0, 0]
Ftip = [0, 0, 0, 0, 0, 0]

# Scenario 1
thetalist = [0, 0, 0, 0, 0, 0]       # Initial joint angles for scenario 1
recordedData = [thetalist]           # List to store the joint angles at each iteration

# Run the simulation for 300 iterations
for i in range(300):
    dt = 0.001                                                                                             # Time step of 0.001 seconds at each iteration                                                                                                                     
    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)         # Calculate the joint accelerations  
    [thetalistNext, dthetalistNext] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)                 # Update the joint angles and velocities      
    thetalist = thetalistNext                                                                              # Update the joint angles
    dthetalist = dthetalistNext                                                                            # Update the joint velocities
    recordedData.append(thetalist)                                                                         # Record the joint angles
    # print(f"Iteration {i+1}, thetalist: {thetalist}")                                                   ## Print the joint angles at each iteration

pd.DataFrame(recordedData).to_csv("simulation1.csv", index=False)                                         # Save the recorded data to a CSV file

# Scenario 2
thetalist = [0, -1, 0, 0, 0, 0]      # Initial joint angles for scenario 2
recordedData = [thetalist]           # List to store the joint angles at each iteration

for i in range(500):
    dt = 0.001   
    ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
    [thetalistNext, dthetalistNext] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
    thetalist = thetalistNext
    dthetalist = dthetalistNext
    recordedData.append(thetalist)
    # print(f"Iteration {i+1}, thetalist: {thetalist}")
    
pd.DataFrame(recordedData).to_csv("simulation2.csv", index=False)

print("Data saved to ch8_scenario1.csv and ch8_scenario2.csv")