import matplotlib.pyplot as plt
import numpy as np
import math


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def read_pose_data(filename):
    """Read pose data from a text file."""
    x = []
    y = []
    orientation = []

    with open(filename, 'r') as file:
        for line in file:
            # Split line into different parts
            parts = line.split()
            # Save x val
            x.append(float(parts[1]))
            # Save y val
            y.append(float(parts[2]))
            # Save orientation
            roll, pitch, yaw = quaternion_to_euler_angle(
                float(parts[7]), float(parts[4]), float(parts[5]), float(parts[6]))
            orientation.append(yaw)

    return x, y, orientation


# Read data from text files
x1, y1, orientation1 = read_pose_data('results/KCC_Keyframe.txt')
x2, y2, orientation2 = read_pose_data('results/optimized_keyframe.txt')

answer = False
if answer is True:
    # Plot the poses
    plt.figure(figsize=(10, 6))
    plt.plot(x1, y1, 'ro', label='KCC Keyframe')
    plt.plot(x2, y2, 'go', label='Optimized Keyframe')

    # Plot points with orientation using arrows
    for i in range(len(x1)):
        plt.arrow(x1[i], y1[i], 0.1*np.cos(orientation1[i]), 0.1*np.sin(orientation1[i]),
                  head_width=0.01, head_length=0.02, fc='b', ec='r')
    for i in range(len(x2)):
        plt.arrow(x2[i], y2[i], 0.1*np.cos(orientation2[i]), 0.1*np.sin(orientation2[i]),
                  head_width=0.01, head_length=0.02, fc='r', ec='g')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Poses with Orientation')
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    # Plot the poses
    plt.figure(figsize=(10, 6))
    plt.plot(x1, y1, label='KCC Keyframe', color='r')
    plt.plot(x2, y2, label='Optimized Keyframe', color='g')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Pose Variation over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
