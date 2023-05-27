function f_p = ff_p(in1,in2,in3)
%FF_P
%    F_P = FF_P(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    23-May-2023 17:31:52

b = in3(2,:);
r = in3(1,:);
theta = in1(3,:);
wl = in2(2,:);
wr = in2(1,:);
t2 = cos(theta);
t3 = sin(theta);
t4 = 1.0./b;
t5 = t4.^2;
f_p = reshape([(t2.*wl)./2.0+(t2.*wr)./2.0,(t3.*wl)./2.0+(t3.*wr)./2.0,t4.*wl.*(-1.0./2.0)+(t4.*wr)./2.0,0.0,0.0,(r.*t5.*wl)./2.0-(r.*t5.*wr)./2.0],[3,2]);
end