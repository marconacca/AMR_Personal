function g_a_4y_ = g_a_4y(in1,t,k_v,k_p)
%G_A_4Y
%    G_A_4Y_ = G_A_4Y(IN1,T,K_V,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-May-2023 12:08:53

theta = in1(3,:);
t2 = sin(theta);
g_a_4y_ = [(t2.*(k_v+k_p.*t))./(cos(theta).^2+t2.^2);0.0;t];
end
