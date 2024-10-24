def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    # TODO - fill out "T"
    T = np.vstack((np.hstack((R, p.reshape(-1, 1))), np.array([0, 0, 0, 1])))

    return T

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    #TODO - fill this out 
    R = T[:3, :3]
    p = T[3:, :3]
    R_inv = np.transpose(R)
    p_inv = -R_inv @ p
    T_inv = np.vstack(np.hstack(R_inv,p_inv),np.array([0,0,0,1]))

    return T_inv
