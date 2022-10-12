% 旋转过程：机体坐标系向NED坐标系旋转
syms phi theta psi;

% 绕Z轴旋转 从机载NED向机体坐标轴系旋转
C_n_b_z = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1];
% 绕Y轴旋转 从机载NED向机体坐标轴系旋转
C_n_b_y = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)];
% 绕X轴旋转 从机载NED向机体坐标轴系旋转
C_n_b_x = [1, 0, 0; 0, cos(phi), sin(phi); 0, -sin(phi), cos(phi)];
% 航空次序欧拉角，按照Z-Y-X的顺序进行旋转 r'=R(x)*R(y)*R(z)*r
% 由机载NED坐标系向机体轴坐标系转换的转换矩阵 该矩阵为正交矩阵
C_n_b = C_n_b_x * C_n_b_y * C_n_b_z;
% 由机体轴坐标系向机载NED坐标系转换的转换矩阵 该矩阵与机载NED坐标系向机体轴坐标系的旋转矩阵互为转置（正交）
C_b_n = C_n_b';

% 漂移过程：计算施加在机体上的力使其在地理坐标系下的位置变化
syms f1 f2 f3 f4;
% 机体模型质量与重力加速度
syms m g;
% 外部控制量
u1 = f1 + f2 + f3 + f4;
a_all = (C_b_n * [0; 0; u1] - [0; 0; m*g]) ./ m;

% 四元数表征旋转过程
syms q0 q1 q2 q3;
q = [q0 q1 q2 q3];
R_b_n_q = [ q0.^2+q1.^2-q2.^2-q3.^2,	2*(q1*q2-q0*q3),			2*(q1*q3+q0*q2);
			2*(q1*q2+q0*q3),			q0.^2-q1.^2+q2.^2-q3.^2,	2*(q2*q3-q0*q1);
			2*(q1*q3-q0*q2),			2*(q2*q3+q0*q1),			q0.^2-q1.^2-q2.^2+q3.^2 ];

% 旋转过程：以四轴十字分布机型为例
% 定义绕三个轴旋转的转动惯量Jx，Jy，Jz，J=mr^2，为质点m对转轴O的转动惯量
% 力矩M = r x F = F*r*sin(theta) = F*d
% 设单个质点m与转轴刚性连接 M = r*F*sin(theta) = r*Ft = r*(m * a_t) = r*m*(r*alpha) = m*r^2*alpha = J*alpha (alpha为角加速度)
syms Jxx Jyy Jzz;
syms L b;
syms phi_vel theta_vel psi_vel;
I = [Jxx, 0, 0; 0, Jyy, 0; 0, 0, Jzz];
phi_acc = (theta_vel * psi_vel * (Jyy - Jzz) + L * (f1 - f3)) / Jxx;
theta_acc = (phi_vel * psi_vel * (Jzz - Jxx) + L * (f2 - f4)) / Jyy;
psi_acc = (theta_vel * phi_vel * (Jxx - Jyy) + b * (f1 - f2 + f3 - f4)) / Jzz;

%{
上述作用过程的顺序为：
	1.计算四个电机处产生的力
	2. 计算这四个力在当前角度下产生的XYZ三个方向的加速度
	3. 计算这四个力对当前机体产生的转动力矩
	4. 转动力矩使得飞机姿态发生变化
	5. 计算飞机当前的位置
%}

% 计算结果
% C_b_n C_n_b a_all q R_b_n_q phi_acc theta_acc psi_acc

Clock = 0 : 0.01 : 10;