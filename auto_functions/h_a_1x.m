function h_a_1x_ = h_a_1x(in1,t,in3,in4,k_v,k_p)
%H_A_1X
%    H_A_1X_ = H_A_1X(IN1,T,IN3,IN4,K_V,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-May-2023 12:08:50

b = in4(2,:);
r = in4(1,:);
theta = in1(3,:);
xhi_v = in3(1,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = t.^2;
t5 = t.^3;
t9 = 1.0./r;
t6 = t4.^2;
t7 = t2.^2;
t8 = t3.^2;
t13 = t4.*1.2e+1;
t14 = k_v.*t5.*4.0;
t10 = k_p.*t6;
t11 = t7.*xhi_v;
t12 = t8.*xhi_v;
t15 = t11+t12;
t17 = t10+t13+t14;
t16 = 1.0./t15;
t18 = b.*t3.*t9.*t16.*t17;
h_a_1x_ = [-t18;t18];
end
