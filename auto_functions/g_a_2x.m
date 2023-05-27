function g_a_2x_ = g_a_2x(in1,t,k_v,k_p)
%G_A_2X
%    G_A_2X_ = G_A_2X(IN1,T,K_V,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:20

theta = in1(3,:);
t2 = cos(theta);
t3 = t.^3;
g_a_2x_ = [(t2.*(t.*6.0+k_p.*t3+k_v.*t.^2.*3.0))./(sin(theta).^2+t2.^2);t3;0.0];
end
