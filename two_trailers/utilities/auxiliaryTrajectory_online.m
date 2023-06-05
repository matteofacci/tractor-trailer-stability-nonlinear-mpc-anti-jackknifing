function [x_final,y_final,theta_final,psi_final,psi2_final,phi_final,x_dot_final,y_dot_final] = auxiliaryTrajectory_online(motion,model,x0,t_aux,x_aux,y_aux,n_s,K1,K2)

global L L1 L2 d N T

% IO feedback linearization

[dx, ~] = firstsecondderivatives(t_aux,x_aux);
[dy, ~] = firstsecondderivatives(t_aux,y_aux);

if motion == "backward"

    % Genero i riferimenti da fornire al sistema I-O linearizzato per generare
    % la traiettoria desiderata di stato.
    x_frwd = flip(x_aux);
    y_frwd = flip(y_aux);
    dx_frwd = -flip(dx);
    dy_frwd = -flip(dy);

else
    % Genero i riferimenti da fornire al sistema I-O linearizzato per generare
    % la traiettoria desiderata di stato.
    x_frwd = x_aux;
    y_frwd = y_aux;
    dx_frwd = dx;
    dy_frwd = dy;

end

% tau è la tangente della curva nel punto finale.
% Per generare la traiettoria desiderata di stato, i valori iniziali del
% tractor heading angle e trailer heading angle (quindi psi = 0)
% verranno posti uguali all'inclinazione della tangente tau.
tau = atan2(dy_frwd, dx_frwd);
indx = dx==0 & dy==0;
tau(indx) = tau(find(~indx,1,'last'));

% GENERO I RIFERIMENTI PER LO STATO
q0 = [x_frwd(1);y_frwd(1);tau(1);0;0;0];
%q0 = [0;0;0;0;0];


%[t,q] = ode45(@(t,q) mysystem(t,q,x_frwd,dx_frwd,y_frwd,dy_frwd,t_aux),tspan,q0);
%[t,q] = ode45(@(t,q) sistema(t,q,dx_frwd,dy_frwd,t_aux),tspan,q0);
% I riferimenti sono in ordine inverso, per ottenerli nel giusto ordine
% inverto le matrici con l'operazione flip di Matlab:
q = zeros(length(t_aux),n_s);
q(1,:) = q0';

for i=1:length(t_aux)-1
    u1 = dx_frwd(i)-K1*(q(i,1)-x_frwd(i));
    u2 = dy_frwd(i)-K2*(q(i,2)-y_frwd(i));
    [t,state] = ode45(@(t,state) IO_linearized_model(t,state,u1,u2),[0,T],q(i,:)');

    q(i+1,:) = state(end,:);
end

q = q(end-N+1:end,:);

if motion == "backward"
    q = flip(q);
end

con_x = q(:,1);
con_y = q(:,2);
con_theta = q(:,3);
con_psi = q(:,4);
con_psi2 = q(:,5);
con_phi = q(:,6);

[con_x_dot, ~] = firstsecondderivatives(t_aux(1:N),con_x);
[con_y_dot, ~] = firstsecondderivatives(t_aux(1:N),con_y);

% Wrapping theta
angle_diff_theta = mod(wrapTo2Pi(x0(3)) - wrapTo2Pi(con_theta(1)) + pi, 2*pi) - pi; % signed angle difference

realFirst_theta = x0(3)-angle_diff_theta;
con_theta = con_theta-con_theta(1);
con_theta = unwrap(con_theta+realFirst_theta);

if model == "standardrwdtractortrailer"

    xR_vec = con_x - L*cos(con_theta) - d*cos(con_theta+con_phi);
    yR_vec = con_y - L*sin(con_theta) - d*sin(con_theta+con_phi);

    [~,index] = nearestPoint(xR_vec,yR_vec,con_x(end),con_y(end));

    x_final = con_x(index);
    y_final = con_y(index);
    theta_final = con_theta(index);
    psi_final = con_psi(index);
    psi2_final = con_psi2(index);
    phi_final = con_phi(index);
    x_dot_final = con_x_dot(index);
    y_dot_final = con_y_dot(index);

else
    x_final = con_x(end);
    y_final = con_y(end);
    theta_final = con_theta(end);
    psi_final = con_psi(end);
    psi2_final = con_psi2(end);
    phi_final = con_phi(end);
    x_dot_final = con_x_dot(end);
    y_dot_final = con_y_dot(end);
end

end