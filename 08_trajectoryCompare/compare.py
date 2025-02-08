import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob  # 用于查找符合特定模式的文件路径
import os  # 用于提取文件名

# 定义文件路径模式（假设所有CSV文件都在当前目录下，且文件名以 'poses_' 开头）
file_pattern = 'poses_*.csv'  # 替换为你的文件路径模式

# 获取所有符合条件的文件路径
file_paths = glob.glob(file_pattern)

# 创建3D图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 定义颜色和标记样式列表，用于区分不同的轨迹
colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
markers = ['o', '^', 's', 'D', 'x', '+', '*']

# 遍历所有文件并绘制轨迹
for i, file_path in enumerate(file_paths):
    # 读取CSV文件，忽略第一行数据
    data = pd.read_csv(file_path, usecols=[1, 2, 3], names=['x', 'y', 'z'], skiprows=1)
    
    # 提取文件名中 '_' 后面的部分作为标签
    file_name = os.path.basename(file_path)  # 获取文件名（不包括路径）
    label = file_name.split('_', 1)[-1].rsplit('.', 1)[0]  # 提取 '_' 后面的部分并去掉扩展名
    
    # 绘制轨迹
    color = colors[i % len(colors)]  # 循环使用颜色列表
    marker = markers[i % len(markers)]  # 循环使用标记样式列表
    ax.plot(data['x'], data['y'], data['z'], c=color, marker=marker, label=label)

# 设置坐标轴标签
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# 显示图例
ax.legend()

# 显示图形
plt.show()