import numpy as np

def rot2quat(rot):
    rot = rot.flatten()
    t = rot[::2].sum()
    r = np.sqrt(1+t)

    w = 0.5*r
    x = copysign(0.5*np.sqrt(1+rot[0]-rot[4]-rot[8]),rot[7]-rot[5])
    y = copysign(0.5*np.sqrt(1-rot[0]+rot[4]-rot[8]),rot[2]-rot[6])
    z = copysign(0.5*np.sqrt(1-rot[0]-rot[4]+rot[8]),rot[3]-rot[1])

    return [x,y,z,w]

def copysign(x,y):
    if y>=0:
        return np.abs(x)
    return -np.abs(x)
