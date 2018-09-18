import sys
import math
import numpy as np
import matplotlib.pyplot as plt

def ResultResolve(path): 
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    with open(path, 'r') as fin:
        line_idx = 0
        
        position = []

        for line in fin:
            is_x_data = line.find('x')
            is_y_data = line.find('y')
            is_z_data = line.find('z')

            if is_x_data != -1:
                x_val = float((line.rstrip().split(':'))[-1])
                                            
            if is_y_data != -1:             
                y_val = float((line.rstrip().split(':'))[-1])
                                            
            if is_z_data != -1:             
                z_val = float((line.rstrip().split(':'))[-1])

            line_idx += 1 

            if 0 == line_idx % 4:
                position.append([x_val, y_val, z_val])
               
        return np.array(position, dtype = np.float)

def GroundTruthDataResolve(path):
    data = np.fromfile(path, dtype = np.float, sep = ' ')
    result = data.reshape([-1, 12])
    return result[:, [3, 7, 11]] 

def Rotation(mat, theta):
    # mat.shape should be 1 * 2
    rot_mat = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    ret = np.dot(mat, rot_mat)
    ret[:, 0] = -ret[:, 0]
    return ret

def Transform(mat, pos_x, pos_y):
    mat[:, 0] = mat[:, 0] - pos_x
    mat[:, 1] = mat[:, 1] - pos_y
    return mat


if __name__=="__main__":
    assert(len(sys.argv) == 3)
    # r_x, r_y, r_z = ResultResolve(sys.argv[1])
    result = ResultResolve(sys.argv[1])
    ground_truth = GroundTruthDataResolve(sys.argv[2])
    # print(result[1, 0])
    # print(result[1, 2])

    mat = ground_truth[:, [0, 2]]
    ret = Rotation(mat, -0.5768)

     
    plt.plot(result[:, 0], result[:, 2], 'r-')
    plt.plot(ret[:, 0],ret[:, 1], 'b-')
    plt.show()


