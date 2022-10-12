clear;

pos = [0; 0; 0];
vel = [0; 0; 0];
ang = [0; 0; 0];
ang_vel = [0; 0; 0];

dt = 0.05;
T = 0 : dt : 1000;

n1 = 1 + 0.001 * T;
n2 = 1 + 0.001 * T;
n3 = 1 + 0.001 * T;
n4 = 1 + 0.001 * T;
n5 = 1 + 0.001 * T;
n6 = 1 + 0.001 * T;
n1(500 : 501) = 0;
n1(700 : 701) = 0;
n1(900 : 901) = 0;
n1(1100 : 1101) = 0;
n1(1600 : 1601) = 0;
n1(1800 : 1801) = 0;

len = length(n1);

pos_recoder = nan(len, 3);
vel_recoder = nan(len, 3);
ang_recoder = nan(len, 3);
ang_vel_recoder = nan(len, 3);

h = animatedline(pos(1), pos(2), pos(3));
h.Color = 'red';
h.LineWidth = 2;

for i = 1 : len
	tau_thrust = dynamical_system_model(n1(i), n2(i), n3(i), n4(i), n5(i), n6(i));
	[ang, ang_vel, ~] = rigid_body_dynamics_model(tau_thrust, ang_vel, ang, dt);
	[pos, vel, ~] = rigid_body_kinematics_model(ang, tau_thrust, pos, vel, dt);
	
	pos_recoder(i, :) = pos';
	vel_recoder(i, :) = vel';
	ang_recoder(i, :) = ang';
	ang_vel_recoder(i, :) = ang_vel';
	
	addpoints(h, pos(1), pos(2), -pos(3));
	drawnow limitrate
end
pause(3);

figure(1);
plot3(pos_recoder(:, 1), pos_recoder(:, 2), -pos_recoder(:, 3), 'r', 'LineWidth', 2); hold on;
title('NED position: X-Y-Z'); xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; grid minor;

figure(2);
plot(vel_recoder(:, 1), 'r', 'LineWidth', 2); hold on;
plot(vel_recoder(:, 2), 'b', 'LineWidth', 2); hold on;
plot(vel_recoder(:, 3), 'c', 'LineWidth', 2); hold on;
title('NED velocity: X-Y-Z');
grid on; grid minor;

figure(3);
plot(ang_recoder(:, 1) * 57.3, 'r', 'LineWidth', 2); hold on;
plot(ang_recoder(:, 2) * 57.3, 'b', 'LineWidth', 2); hold on;
plot(ang_recoder(:, 3) * 57.3, 'c', 'LineWidth', 2); hold on;
title('angle');
grid on; grid minor;

figure(4);
plot(ang_vel_recoder(:, 1) * 57.3, 'r', 'LineWidth', 2); hold on;
plot(ang_vel_recoder(:, 2) * 57.3, 'b', 'LineWidth', 2); hold on;
plot(ang_vel_recoder(:, 3) * 57.3, 'c', 'LineWidth', 2); hold on;
title('angular velocity');
grid on; grid minor;
