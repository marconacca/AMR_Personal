function func = dhq_a3x(b,k_p,k_v,r,t,theta,xhi_v)
%DHQ_A3X
%    FUNC = DHQ_A3X(B,K_P,K_V,R,T,THETA,XHI_V)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-May-2023 12:08:54

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
t15 = b.*t2.*t8.*t12.*t14;
func = reshape([0.0,0.0,0.0,0.0,-t15,t15],[2,3]);
end
