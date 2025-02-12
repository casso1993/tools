import numpy as np
import quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def imu_integration(timestamps, gyro_data, accel_data):
    # 初始化初始时刻的重力向量和姿态
    init_accel = accel_data[0]
    g = np.linalg.norm(init_accel)
    a_init_normalized = init_accel / g

    # 构造初始四元数，将重力对齐到世界坐标系Z轴
    v1 = np.array([0.0, 0.0, 1.0])
    v2 = a_init_normalized
    q_init = quaternion_from_vectors(v1, v2)

    # 初始化状态变量
    current_q = q_init
    velocity = np.zeros(3)
    position = np.zeros(3)
    positions = [position.copy()]

    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        if dt <= 0 or dt > 0.1:
            continue

        # 角速度积分更新姿态
        w = gyro_data[i-1]
        delta_theta = w * dt
        delta_q = quaternion.from_rotation_vector(delta_theta)
        current_q = (current_q * delta_q).normalized()

        # 加速度处理
        a_body = accel_data[i]
        a_world = rotate_vector(current_q, a_body)
        a_motion = a_world - np.array([0.0, 0.0, g])

        # 速度与位置积分
        velocity += a_motion * dt
        position += velocity * dt + 0.5 * a_motion * dt**2
        positions.append(position.copy())

    return np.array(positions)

def quaternion_from_vectors(vec1, vec2):
    """Return the quaternion rotating vec1 to align with vec2."""
    vec1 /= np.linalg.norm(vec1)
    vec2 /= np.linalg.norm(vec2)
    cos_theta = np.dot(vec1, vec2)
    if cos_theta == -1:
        axis = np.cross([1, 0, 0], vec1)
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross([0, 1, 0], vec1)
        axis /= np.linalg.norm(axis)
        angle = np.pi
    else:
        axis = np.cross(vec1, vec2)
        angle = np.arccos(cos_theta)
    half_angle = angle / 2
    sin_half_angle = np.sin(half_angle)
    return quaternion.quaternion(np.cos(half_angle), *(axis * sin_half_angle))

def rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    v_quat = quaternion.quaternion(0, *v)
    rotated_v_quat = q * v_quat * q.conjugate()
    return np.array([rotated_v_quat.x, rotated_v_quat.y, rotated_v_quat.z])

def read_imu_data(file_path):
    data = np.loadtxt(file_path)
    timestamps = data[:, 0]
    accel_data = data[:, 1:4]
    gyro_data = data[:, 4:7]
    return timestamps, gyro_data, accel_data

def plot_positions(positions):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend()
    plt.title('IMU Integrated Trajectory')
    plt.show()

# 示例：假设数据存储在 'imu_data.txt' 文件中
file_path = '/home/casso/tools/04_imu2txt/txt/imu.txt'
timestamps, gyro_data, accel_data = read_imu_data(file_path)
positions = imu_integration(timestamps, gyro_data, accel_data)
plot_positions(positions)



