function g_a_5x_ = g_a_5x(in1,k_p)
%G_A_5X
%    G_A_5X_ = G_A_5X(IN1,K_P)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-May-2023 12:08:53

theta = in1(3,:);
t2 = cos(theta);
g_a_5x_ = [(k_p.*t2)./(sin(theta).^2+t2.^2);1.0;0.0];
end
