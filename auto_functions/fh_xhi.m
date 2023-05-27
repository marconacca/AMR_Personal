function h_xhi = fh_xhi(in1,in2,in3,in4,in5,in6,k_v,k_i,k_p)
%FH_XHI
%    H_XHI = FH_XHI(IN1,IN2,IN3,IN4,IN5,IN6,K_V,K_I,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:31:53

b = in6(2,:);
ddx_d = in5(1,:);
ddy_d = in5(2,:);
dx_d = in4(1,:);
dy_d = in4(2,:);
r = in6(1,:);
theta = in1(3,:);
x = in1(1,:);
x_d = in3(1,:);
xhi_v = in2(1,:);
xhi_x = in2(2,:);
xhi_y = in2(3,:);
y = in1(2,:);
y_d = in3(2,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = k_i.*xhi_x;
t5 = k_i.*xhi_y;
t10 = 1.0./r;
t11 = -x_d;
t12 = -y_d;
t6 = t2.^2;
t7 = t3.^2;
t8 = t2.*xhi_v;
t9 = t3.*xhi_v;
t15 = t11+x;
t17 = t12+y;
t13 = t6.*xhi_v;
t14 = t7.*xhi_v;
t16 = -t8;
t18 = -t9;
t20 = k_p.*t15;
t21 = k_p.*t17;
t23 = t6+t7;
t19 = dx_d+t16;
t22 = dy_d+t18;
t26 = -t20;
t27 = -t21;
t28 = t13+t14;
t24 = k_v.*t19;
t25 = k_v.*t22;
t29 = 1.0./t28;
t30 = t29.^2;
t31 = ddx_d+t4+t24+t26;
t32 = b.*k_i.*t2.*t10.*t29;
t33 = ddy_d+t5+t25+t27;
t34 = b.*k_i.*t3.*t10.*t29;
t35 = t3.*t23.*t30.*t31;
t36 = t2.*t23.*t30.*t33;
t37 = -t36;
t38 = t35+t37;
t39 = b.*t10.*t38;
h_xhi = reshape([t10+t39,t10-t39,-t34,t34,t32,-t32],[2,3]);
end