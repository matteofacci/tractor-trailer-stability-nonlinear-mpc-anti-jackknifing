function dstate = IO_linearized_model(t, state, u1, u2)

global L L1 L2 L3 L4 d

theta = state(3);
psi = state(4);
psi2 = state(5);
phi = state(6);

dstate = [ u1; ...
    u2; ...
    sin(phi)/L*(cos(theta+phi)*u1 + sin(theta+phi)*u2); ...
    -1/(L*L2)*(L1*sin(phi)*cos(psi) + L2*sin(phi) + L*cos(phi)*sin(psi))*(cos(theta+phi)*u1 + sin(theta+phi)*u2); ...
    ((L2*sin(phi)/L*L4)*((L4*cos(psi)/L2)+((L2+L3)*cos(psi)*cos(psi2)/L2)*cos(psi+psi2))+(cos(phi)*sin(psi)/L2)*(1+((L2+L3)*cos(psi2)/L4))-(cos(phi)*sin(psi+psi2)/L4))*(cos(theta+phi)*u1+sin(theta+phi)*u2); ...
    -u1*(cos(theta+phi)*sin(phi)/L + sin(theta+phi)/d) - u2*(sin(theta+phi)*sin(phi)/L - cos(theta+phi)/d) ];
% - u2*(sin(theta+phi)*sin(phi)/L - cos(theta+phi)/d)
%    -(1/(L*L2))*(L*cos(phi)*sin(psi)+L2*sin(phi)*cos(psi)+L2*sin(phi))(cos(theta+phi)*u1+sin(theta+phi)*u2);


