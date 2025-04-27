import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义位姿矩阵 T_matrix_init
T_matrix_init = np.array([
    [0.99962538779588206, 0.00017851420666272099, 0.027368818144203073, 0],
    [0.00017851420666272099, 0.99991493250452035, -0.013042081396713631, 0],
    [-0.027368818144203073, 0.013042081396713631, 0.99954032030040241, 0],
    [5, 0, 0, 1]
])

# 提取旋转部分和位移
rotation = T_matrix_init[:3, :3]  # 旋转矩阵
translation = T_matrix_init[:3, 3]  # 位移向量

# 创建坐标轴
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制坐标轴
origin = translation
x_axis = origin + rotation[:, 0]  # X 轴方向
y_axis = origin + rotation[:, 1]  # Y 轴方向
z_axis = origin + rotation[:, 2]  # Z 轴方向

# 绘制坐标系
ax.quiver(*origin, *(x_axis - origin), color='r', length=1, normalize=True, label='X-axis')
ax.quiver(*origin, *(y_axis - origin), color='g', length=1, normalize=True, label='Y-axis')
ax.quiver(*origin, *(z_axis - origin), color='b', length=1, normalize=True, label='Z-axis')

# 设置图形属性
ax.scatter(*origin, color='k', s=100)  # 绘制位姿点
ax.set_xlim([-1, 6])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Pose Visualization')
ax.legend()

# 显示图形
plt.show()