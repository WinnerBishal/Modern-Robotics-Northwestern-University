import numpy as np
import modern_robotics as mr


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot
    
    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    
    maxiterations = 20
    se3mat = mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                      thetalist)), T))
    Vb = mr.se3ToVec(se3mat)
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
          
    joints_for_csv = np.array(thetalist)
    
    while err and i < maxiterations:
        print(f"Iteration {i}:")
        
        # Update thetalist
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        thetalist = [x % (2 * np.pi) for x in thetalist]                                            # To ensure thetalist is within the radians range            
        print("Joint vector: " + ", ".join([f'{x:.3f}' for x in thetalist]))
        
        # Calculate end effector configuration
        Tsb = mr.FKinBody(M, Blist, thetalist)
        print("SE(3) end-effector config: " + " ; ".join([" ".join([f'{x:.3f}' for x in row]) for row in Tsb]))
        
        # Calculate se3 representation of twist
        se3mat = mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T))
        Vb = mr.se3ToVec(se3mat)
        print('error twist V_b:' + ', '.join([f'{x:.3f}' for x in Vb]))
        
        # Calculate Errors
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        print(f"angular error magnitude ∣∣omega_b∣∣: {np.linalg.norm([Vb[0], Vb[1], Vb[2]]):.3f}")
        print(f"linear error magnitude ||e_v||: {np.linalg.norm([Vb[3], Vb[4], Vb[5]]):.3f}\n")
        
        # Update Iteration
        i = i + 1
        
        joints_for_csv = np.vstack((joints_for_csv, thetalist))
        
        # Ensure joint values are truncated to 3 decimal places
        np.savetxt("joints.csv", joints_for_csv, delimiter=",", fmt='%.3f')
        
    return (thetalist, not err)

# Example Execution

# Blist = np.array([[0, 1, 0, 0.191, 0, 0.817],
#                   [0, 0,  1, 0.095, -0.817, 0],
#                   [0, 0,  1, 0.095, -0.392, 0], 
#                   [0, 0, 1, 0.095, 0, 0],
#                   [0, -1, 0, -0.082, 0, 0],
#                   [0, 0, 1, 0, 0, 0]]).T

# M = np.array([[-1, 0,  0, 0.817],
#               [ 0, 0,  1, 0.191],
#               [ 0, 1, 0, -0.006],
#               [ 0, 0,  0, 1]])

# T = np.array([[0, 1,  0, -0.5],
#               [0, 0, -1, 0.1],
#               [-1, 0, 0, 0.1],
#               [0, 0,  0,   1]])
# thetalist0 = np.array([5, -2, -2, -2, 2.5, 4])
# eomg = 0.001
# ev = 0.0001

# IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)