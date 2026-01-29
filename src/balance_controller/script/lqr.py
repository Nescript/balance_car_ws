import numpy as np
import control

# 物理参数
M = 1.0     # 轮子质量 (0.5kg * 2)
m = 2.0     # 车身质量 (mass=2.0)
l = 0.122   # 轮轴到重心距离 (wheel_joint z偏移绝对值 = height*2 = 0.061*2)
g = 9.81    # 重力加速度
I = 0.04229  # 车身绕 Y 轴转动惯量 (iyy)
b = 0.02     # 摩擦系数 (从 xacro damping=0.01 得到)
r = 0.1


# 状态空间矩阵 A 和 B
Mt = M + m
Jt = I + m * (l**2)
delta = Mt * Jt - (m*l)**2

# 状态向量 x = [p, v, theta, omega]^T
# 矩阵 A (4x4)
A = np.array([
    [0, 1, 0, 0],
    [0, -(Jt*b)/delta, -(m**2 * g * l**2)/delta, 0],
    [0, 0, 0, 1],
    [0, -(m*l*b)/delta, (Mt*m*g*l)/delta, 0]
])

# 矩阵 B (4x1)
B = np.array([
    [0],
    [Jt/(delta * r)],     
    [0],
    [-(m*l)/(delta * r)]
])


# Q 和 R
Q = np.diag([
    9000,    # 位置权重
    600,      # 速度权重
    700,    # 角度权重
    5       # 角速度权重
])

# 控制权重
R = 100

K, S, E = control.lqr(A, B, Q, R)

print(f"  k1: {K[0,0]:.4f}\n  k2: {K[0,1]:.4f} \n  k3: {K[0,2]:.4f} \n  k4: {K[0,3]:.4f}")