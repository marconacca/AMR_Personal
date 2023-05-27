function h_a_3x_ = h_a_3x(in1,t,in3,in4,k_v,k_p)
%H_A_3X
%    H_A_3X_ = H_A_3X(IN1,T,IN3,IN4,K_V,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:19

b = in4(2,:);
r = in4(1,:);
theta = in1(3,:);
xhi_v = in3(1,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = t.^2;
t6 = k_v.*t.*2.0;
t8 = 1.0./r;
t5 = t2.^2;
t7 = t3.^2;
t9 = k_p.*t4;
t10 = t5.*xhi_v;
t11 = t7.*xhi_v;
t12 = t6+t9+2.0;
t13 = t10+t11;
t14 = 1.0./t13;
t15 = b.*t3.*t8.*t12.*t14;
h_a_3x_ = [-t15;t15];
end