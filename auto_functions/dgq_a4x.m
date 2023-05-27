function func = dgq_a4x(k_p,k_v,t,theta)
%DGQ_A4X
%    FUNC = DGQ_A4X(K_P,K_V,T,THETA)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:41

t2 = sin(theta);
func = reshape([0.0,0.0,0.0,0.0,0.0,0.0,-(t2.*(k_v+k_p.*t))./(cos(theta).^2+t2.^2),0.0,0.0],[3,3]);
end
