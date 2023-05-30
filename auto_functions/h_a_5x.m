function h_a_5x_ = h_a_5x(in1,in2,in3,k_p)
%H_A_5X
%    H_A_5X_ = H_A_5X(IN1,IN2,IN3,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-May-2023 12:08:52

b = in3(2,:);
r = in3(1,:);
theta = in1(3,:);
xhi_v = in2(1,:);
t2 = cos(theta);
t3 = sin(theta);
t6 = 1.0./r;
t4 = t2.^2;
t5 = t3.^2;
t7 = t4.*xhi_v;
t8 = t5.*xhi_v;
t9 = t7+t8;
t10 = 1.0./t9;
t11 = b.*k_p.*t3.*t6.*t10;
h_a_5x_ = [-t11;t11];
end
