function func = dgq_a3y(k_p,k_v,t,theta)
%DGQ_A3Y
%    FUNC = DGQ_A3Y(K_P,K_V,T,THETA)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:32:47

t2 = cos(theta);
func = reshape([0.0,0.0,0.0,0.0,0.0,0.0,(t2.*(k_v.*t.*2.0+k_p.*t.^2+2.0))./(sin(theta).^2+t2.^2),0.0,0.0],[3,3]);
end