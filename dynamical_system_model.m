function tau_thrust = dynamical_system_model(n1, n2, n3, n4, n5, n6)
	c_T = 1.00;		% 升力系数
	c_M = 0.01;		% 扭力系数
	d = 0.50;		% 电机到机体中心的距离
	
	M = [1.0*d*c_T   -1.0*d*c_T    -0.5*d*c_T      0.5*d*c_T      0.5*d*c_T      -0.5*d*c_T;
			 0			  0		  0.866*d*c_T   -0.866*d*c_T    0.866*d*c_T    -0.866*d*c_T;
		-1.0*d*c_M    1.0*d*c_M    -1.0*d*c_M      1.0*d*c_M      1.0*d*c_M      -1.0*d*c_M;
		  -1.0*c_T     -1.0*c_T      -1.0*c_T	    -1.0*c_T       -1.0*c_T        -1.0*c_T];

	n = [n1; n2; n3; n4; n5; n6];
	tau_thrust = M * n;
end