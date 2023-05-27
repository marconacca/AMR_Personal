function h_a_5y_ = h_a_5y(in1,in2,in3,k_p)
%H_A_5Y
%    H_A_5Y_ = H_A_5Y(IN1,IN2,IN3,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:20

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
t11 = b.*k_p.*t2.*t6.*t10;
h_a_5y_ = [t11;-t11];
end
