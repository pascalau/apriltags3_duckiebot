#Various functions to easily manipulate quaternions.

import numpy as np

# Changes rotation representation from a 3x3 rotation matrix to equivalent quaternion.
def rot2quat(rot):
    t = rot.trace()
    rot = rot.flatten()
    r = np.sqrt(1+t)

    w = 0.5*r
    x = copysign(0.5*np.sqrt(1+rot[0]-rot[4]-rot[8]),rot[7]-rot[5])
    y = copysign(0.5*np.sqrt(1-rot[0]+rot[4]-rot[8]),rot[2]-rot[6])
    z = copysign(0.5*np.sqrt(1-rot[0]-rot[4]+rot[8]),rot[3]-rot[1])

    return [x,y,z,w]

#takes the absolute value of the first argument with the sign of the second
def copysign(x,y):
    if y>=0:
        return np.abs(x)
    return -np.abs(x)

#Function to multiply to quaternions together
def quatmul(a_quat,b_quat):
    x = a_quat[3]*b_quat[0]+a_quat[0]*b_quat[3]+a_quat[1]*b_quat[2]-a_quat[2]*b_quat[1]
    y = a_quat[3]*b_quat[1]+a_quat[1]*b_quat[3]+a_quat[2]*b_quat[0]-a_quat[0]*b_quat[2]
    z = a_quat[3]*b_quat[2]+a_quat[2]*b_quat[3]+a_quat[0]*b_quat[1]-a_quat[1]*b_quat[0]
    w = a_quat[3]*b_quat[3]-a_quat[0]*b_quat[0]-a_quat[1]*b_quat[1]-a_quat[2]*b_quat[2]
    return [x,y,z,w]

#calculates the absolute value of a quaternion
def quatabs(quat):
    return np.sqrt(quat[0]**2+quat[1]**2+quat[2]**2+quat[3]**2)

#calculates the complex conjugated of a quaternion
def quatconj(quat):
    return [-quat[0],-quat[1],-quat[2],quat[3]]

#calculates the complex inverse of a quaternion
def quatinv(quat):
    return quatconj(quat)/(quatabs(quat)**2)

#test find the correct transfomation of the old node
if __name__ == '__main__':
    quat = [0.026,-0.038,-0.006,0.999]
    rot = [1,0,0,0]

    print(quatmul(quatmul(rot,quat),quatinv(rot)))
    
    print(quatmul(quat,rot))
