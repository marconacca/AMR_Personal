function func = dhq_a2y(b,k_p,k_v,r,t,theta,xhi_v)
%DHQ_A2Y
%    FUNC = DHQ_A2Y(B,K_P,K_V,R,T,THETA,XHI_V)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:26

t2 = cos(theta);
t3 = sin(theta);
t4 = t.*6.0;
t5 = t.^2;
t6 = t.^3;
t9 = 1.0./r;
t7 = t2.^2;
t8 = t3.^2;
t10 = k_p.*t6;
t13 = k_v.*t5.*3.0;
t11 = t7.*xhi_v;
t12 = t8.*xhi_v;
t15 = t4+t10+t13;
t14 = t11+t12;
t16 = 1.0./t14;
t17 = b.*t3.*t9.*t15.*t16;
func = reshape([0.0,0.0,0.0,0.0,-t17,t17],[2,3]);
end