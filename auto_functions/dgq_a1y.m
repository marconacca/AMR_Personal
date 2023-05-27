function func = dgq_a1y(k_p,k_v,t,theta)
%DGQ_A1Y
%    FUNC = DGQ_A1Y(K_P,K_V,T,THETA)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:43

t2 = cos(theta);
func = reshape([0.0,0.0,0.0,0.0,0.0,0.0,(t2.*(k_p.*t.^4+k_v.*t.^3.*4.0+t.^2.*1.2e+1))./(sin(theta).^2+t2.^2),0.0,0.0],[3,3]);
end