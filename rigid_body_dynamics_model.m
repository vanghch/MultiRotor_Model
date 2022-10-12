function [ang, ang_vel, ang_acc] = rigid_body_dynamics_model(tau_thrust, prev_ang_vel, prev_ang, dt)
	Jxx = 10.0;
	Jyy = 10.0;
	Jzz = 7.50;
	tau = [tau_thrust(1); tau_thrust(2); tau_thrust(3)];
	J = [Jxx 0 0; 0 Jyy 0; 0 0 Jzz];	% 机体惯量矩阵，一般来说飞机的惯量分布是独立的
	
	ang_acc = J \ (cross(-prev_ang_vel, J*prev_ang_vel) + tau);
	ang_vel = prev_ang_vel + ang_acc .* dt;
	ang = prev_ang + prev_ang_vel .* dt + 0.5 .* ang_acc .* dt.^2;
	ang(ang > pi) = ang(ang > pi) - 2*pi;
	ang(ang < -pi) = ang(ang < -pi) + 2*pi;
end