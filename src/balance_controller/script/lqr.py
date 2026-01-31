import numpy as np
import control

# 物理参数
M = 1.0     # 轮子质量 (0.5kg * 2)
m = 3.0     # 车身总质量 (base:2.0 + gimbal:0.2 + muzzle:0.8)
l = 0.0702  # 轮轴到组合重心的垂直距离 (经计算约为 0.07017m)
g = 9.81    # 重力加速度
I = 0.0867  # 车身绕组合重心 Y 轴的总转动惯量 (经平行轴定理计算)
b = 0.02    # 摩擦系数 (两个轮子 damping 0.01 * 2)
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
    1,    # 位置权重
    0.16,      # 速度权重
    1.3759,    # 角度权重
    2      # 角速度权重
])

R = 0.2

K, S, E = control.lqr(A, B, Q, R)

print(f"  k1: {K[0,0]:.4f}\n  k2: {K[0,1]:.4f} \n  k3: {K[0,2]:.4f} \n  k4: {K[0,3]:.4f}")

print(A)
print(B)