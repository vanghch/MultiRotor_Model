function [pos, vel, acc] = rigid_body_kinematics_model(angle, tau_thrust, prev_pos, prev_vel, dt)
	m = 5;
	g = 9.8;
	phi = angle(1);
	theta = angle(2);
	psi = angle(3);
	u1 = [0; 0; tau_thrust(4)];
	global_u = [0; 0; m*g];

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

	acc = ((C_b_n * u1) - global_u) ./ m;
	vel = prev_vel + acc .* dt;
	pos = prev_pos + prev_vel .* dt + 0.5 .* acc .* dt.^2;
end