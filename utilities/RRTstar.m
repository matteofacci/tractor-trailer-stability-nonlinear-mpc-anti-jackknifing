function [q_ref,q_aux,x0,obs,obs_reduced,N_obs,sim_time,v,omega] = RRTstar(motion,model,n_s,K1,K2)

global L L1 L2 d ...
    wheelRGB tractorRGB trailerRGB tractorBodyLenght trailerBodyLenght...
    tractorWidth trailerWidth wheelBase wheelWidth wheelDiam N T

labelFontSize = 18;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultColorbarTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
% set(groot, 'defaultFigureRenderer','painters');
set(groot,'defaultAxesFontSize',labelFontSize)

x_min = 0;
y_min = 0;
x_max = 6;
y_max = 6;

xmin_obs = 1.5; % maximum x coordinate of polygon center
ymin_obs = 1.5; % maximum y coordinate of polygon center
xmax_obs = 5; % maximum x coordinate of polygon center
ymax_obs = 5; % maximum y coordinate of polygon center

safetyExpansion = trailerWidth*2;

% define the velocity of the final trajectory
v = 0.2; % m/s
num_samples = 3000;

EPS = 0.2;
% Within a radius of r, find all existing nodes
r = EPS*4;
numNodes = 1000;
%interpType = 'spline';
%interpType = 'linear';
interpType = 'pchip';

filter = 1;

q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [5.99 5.99];
q_goal.cost = 0;

N_obs = 5; % number of polygons
%N_edges = 5; % number of edges
min_edges = 4; % min number of edges
max_edges = 5; % max number of edges
dim_edge_min = 0.1; % minimum edge dimension value [m]
dim_edge_max = 1; % maximum edge dimension value [m]

% x_coord = [1,3,4,4,6,1];
% y_coord = [2,3,1,5,3,5];

x_coord = [1,3,4,1,6];
y_coord = [2,1,4,5,2];

for i = 1:N_obs

    N_edges = floor(min_edges + (max_edges-min_edges)*rand());
    % generate random dimensions for polygon
    %dimensions = randi([val1 val2], 1, M)/10;
    dimensions = dim_edge_min + (dim_edge_max-dim_edge_min)*rand(1,N_edges);

    % create polygon
    x = dimensions .* cos(2*pi/N_edges*(0:N_edges-1));
    y = dimensions .* sin(2*pi/N_edges*(0:N_edges-1));

    % calculate center of polygon
    % x_center = (xmin_obs + (xmax_obs-xmin_obs)*rand())+(-1 + (1-(-1))*rand());
    % y_center = (ymin_obs + (ymax_obs-ymin_obs)*rand())+(-1 + (1-(-1))*rand());

    x_center = x_coord(i);
    y_center = y_coord(i);

    % shift polygon to center
    x = x + x_center;
    y = y + y_center;

    obs_reduced(i).coord = [x',y'];

    %% Positive expansion
    % expand the polygon by a positive distance (outside of the polygon)
    obs(i).coord = cell2mat(expandPolygon(obs_reduced(i).coord, safetyExpansion));

end

figure(50)
set(gcf, 'Color', 'w');

xlabel('$$x \, [m]$$','Interpreter','latex')
ylabel('$$y \, [m]$$','Interpreter','latex')

for i = 1:N_obs
    drawPolygon(obs_reduced(i).coord,'-k', 'linewidth', 2);
    drawPolygon(obs(i).coord,'--k', 'linewidth', 1);
end
axis([0 x_max 0 y_max])
legend(["Obstacle","Safety expansion"],'Interpreter','latex','Orientation','horizontal','Location','bestoutside','AutoUpdate','off')
%rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
axis([0 x_max 0 y_max])
axis equal
hold on

rad = (0.045*2)/3;
c1 = "k";
c2 = "r";
circles = plotFilledCircle(q_start.coord(1),q_start.coord(2),rad,c1,c2);
label = "Start";
labelpoints(q_start.coord(1),q_start.coord(2),label,'interpreter','latex','buffer',0.5,'FontSize', labelFontSize);
circles = plotFilledCircle(q_goal.coord(1),q_goal.coord(2),rad,c1,c2);
label = "Goal";
labelpoints(q_goal.coord(1),q_goal.coord(2),label,'interpreter','latex','buffer',0.5,'FontSize', labelFontSize);
%rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
hold on

%legend(["Obstacle","Safety expansion","","","",""],'Interpreter','latex','Orientation','horizontal','Location','bestoutside')

axis padded
reverseStr = '';
nodes(1) = q_start;

for i = 1:1:numNodes
    %q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    q_rand = [rand(1)*x_max rand(1)*y_max];
    %plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])

    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end

    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);

    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if noCollision(q_rand, q_near.coord, obs)
        tree_line_plot = line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'b', 'LineWidth', 0.5,'LineStyle','--');
        tree_line_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;

        % Within a radius of r, find all existing nodes
        q_nearest = [];
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obs) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end

        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;

        % Iterate through all nearest neighbors to find alternate lower
        % cost paths

        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obs) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');
                hold on
            end
        end

        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end

        % Append to nodes
        nodes = [nodes q_new];
    end
    msg = sprintf('Processed: %d nodes (Total: %d)', i, numNodes); % display the current iteration
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
i=1;
P(i,:) = q_end.coord;
while q_end.parent ~= 0
    start = q_end.parent;
    % line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    % hold on

    q_end = nodes(start);
    i=i+1;
    P(i,:) = q_end.coord;
end

P = flip(P);
feasability = noCollision(P(end,:), P(end-1,:), obs);

if feasability

    if filter
        % Start the bidirectional search algorithm
        new_P = [];
        currForward = 1;
        while currForward < size(P,1)
            new_P(end+1,:) = P(currForward,:);
            currBackward = size(P,1);
            relaxationFound = false;
            while currBackward ~= currForward && ~relaxationFound
                if noCollision(P(currBackward,:), P(currForward,:), obs)
                    relaxationFound = true;
                else
                    currBackward = currBackward - 1;
                end
            end
            if relaxationFound
                currForward = currBackward;
            else
                currForward = currForward + 1;
            end
        end
        new_P(end+1,:) = P(end,:);

        P = new_P;
    end
    
    if length(P) > 2
        x1 = P(1,1);
        x2 = P(2,1);
        y1 = P(1,2);
        y2 = P(2,2);
        midpoint1 = [(x1+x2)/2,(y1+y2)/2];
        x1 = P(end-1,1);
        x2 = P(end,1);
        y1 = P(end-1,2);
        y2 = P(end,2);
        midpoint2 = [(x1+x2)/2,(y1+y2)/2];

        P = [P(1,:); midpoint1; P(2:end-1,:); midpoint2; P(end,:)];

        %P_temp = [midpoint1; P(2:end-1,:); midpoint2];
        
        %size(P_temp)

        pt = interparc(num_samples,P(:,1),P(:,2),interpType);

        %pt = [P(1,:); pt; P(end,:)];

        %size(pt)

    else

        pt = interparc(num_samples,P(:,1),P(:,2),interpType);
    end
    
    coords = pt;

    distances = sqrt(diff(coords(:,1)).^2 + diff(coords(:,2)).^2);             % calculate the distance between each point
    total_distance = sum(abs(distances)); % calculate the total distance of the path

    times = abs(distances) / v;         % calculate the time to travel each distance at constant velocity
    total_time = sum(times);            % calculate the total time to travel the entire path

    t_interp = 0:T:total_time;  % create the new time intervals

    x_interp = interp1(cumsum([0; times]), [coords(1,1); coords(1:end-1,1)], t_interp); % interpolate the x-coordinates
    y_interp = interp1(cumsum([0; times]), [coords(1,2); coords(1:end-1,2)], t_interp); % interpolate the y-coordinates

    new_coords = [x_interp.', y_interp.']; % combine the x and y coordinates
    new_coords = [new_coords;coords(end,:)];

    for i=1:length(nodes)
        nodes_coord(i,:) = nodes(i).coord;
    end

    tree_scatter = scatter(nodes_coord(:,1),nodes_coord(:,2));
    tree_scatter.Annotation.LegendInformation.IconDisplayStyle = 'off';
    % RRT_plot = plot(P(:,1),P(:,2),'-b','linewidth',2);
    % RRT_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
    % RRT_spline = plot(new_coords(:,1),new_coords(:,2),'--r','linewidth',2);
    % RRT_spline.Annotation.LegendInformation.IconDisplayStyle = 'off';
    fprintf('\n')
else
    fprintf('\n')
    disp("Unfeasible problem");
    fprintf('\n')

    timeout('pause(inf)',1)

end

q_ref = new_coords;

t_aux = 0:T:length(t_interp)*T;

sim_time = t_aux(end);
%t_aux = t_aux(1:end-1);

% t_aux = t_interp;
% sim_time = t_aux(end);

%% Initial alignment

[dx, ~] = firstsecondderivatives(t_aux,q_ref(:,1));
[dy, ~] = firstsecondderivatives(t_aux,q_ref(:,2));

if motion == "backward"
    dx = -dx;
    dy = -dy;
end

for i = length(q_ref)
    q_ref(i,3) = 0;
    q_ref(i,4) = 0;
    q_ref(i,5) = 0;
    % Compute u_ref and omega_ref
    q_ref(i,6) = sqrt(dx(i)^2 + dy(i)^2);
    q_ref(i,7) = (dy(i)*dx(i))/(q_ref(i,1)^2 + q_ref(i,2)^2);
end

[dx, ~] = firstsecondderivatives(t_aux,q_ref(:,1));
[dy, ~] = firstsecondderivatives(t_aux,q_ref(:,2));

% % Compute the derivatives
% dx = diff(P(:,1));
% dy = diff(P(:,2));

if motion == "backward"
    dx = -dx;
    dy = -dy;
end

tau = atan2(dy, dx);
indx = dx==0 & dy==0;
tau(indx) = tau(find(~indx,1,'last'));

x0 = [q_ref(1,1);q_ref(1,2);tau(1);0;0];  % Initial state

x0(3) = wrapTo2Pi(x0(3));


%% IO Linearization

% t_aux = 0:T:num_samples*T+N*T;
% t_aux = t_aux(1:end-1);
x_aux = q_ref(:,1);
y_aux = q_ref(:,2);

% x_aux(num_samples+1:num_samples+N)=q_ref(end,1);
% y_aux(num_samples+1:num_samples+N)=q_ref(end,2);

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
q0 = [x_frwd(1);y_frwd(1);tau(1);0;0];
%q0 = [0;0;0;0;0];


% %[t,q] = ode45(@(t,q) mysystem(t,q,x_frwd,dx_frwd,y_frwd,dy_frwd,t_aux),tspan,q0);
% %[t,q] = ode45(@(t,q) sistema(t,q,dx_frwd,dy_frwd,t_aux),tspan,q0);
% % I riferimenti sono in ordine inverso, per ottenerli nel giusto ordine
% % inverto le matrici con l'operazione flip di Matlab:
% q = zeros(length(t_aux),n_s);
% q(1,:) = q0';
% 
% for i=1:length(t_aux)-1
%     u1 = dx_frwd(i)-K1*(q(i,1)-x_frwd(i));
%     u2 = dy_frwd(i)-K2*(q(i,2)-y_frwd(i));
%     [t,state] = ode45(@(t,state) IO_linearized_model(t,state,u1,u2),[0,T],q(i,:)');
% 
%     q(i+1,:) = state(end,:);
% end
% 
% %q = q(N:num_samples+N,:);
% 
% % q = q(N:end-1,:);
% % t_aux = t_aux(1:num_samples);
% 
% % size(q)
% % size(t_aux)
% if motion == "backward"
%     q = flip(q);
% end

%%
q = zeros(length(t_aux),n_s);
% v = zeros(length(t_aux),1);
% omega = zeros(length(t_aux),1);
q(1,:) = q0';
% v(1) = 0;
% omega(1) = 0;

for i=1:length(t_aux)-1
    u1 = dx_frwd(i)-K1*(q(i,1)-x_frwd(i));
    u2 = dy_frwd(i)-K2*(q(i,2)-y_frwd(i));
    [t,state] = ode45(@(t,state) IO_linearized_model(t,state,u1,u2),[0,T],q(i,:)');
    
    theta = q(i,3);
    phi = q(i,5);

    D = [cos(theta)-(tan(phi)/L)*(L*sin(theta)+d*sin(theta+phi)), -d*sin(theta+phi);
        sin(theta)+(tan(phi)/L)*(L*cos(theta)+d*cos(theta+phi)), d*cos(theta+phi)];
    invD = inv(D);
    
    v(i) = invD(1,:)*[u1;u2];
    omega(i) = invD(2,:)*[u1;u2];

    q(i+1,:) = state(end,:);
end


%q = q(N:total_time-N,:);
% v(length(t_aux)) = 0;
% omega(length(t_aux)) = 0;

if motion == "backward"
    q = flip(q);
    v = -flip(v);
    omega = -flip(omega);
end

v(length(t_aux)) = 0;
omega(length(t_aux)) = 0;

% v = v(1:total_time-2*N);
% omega = omega(1:total_time-2*N);

% con_x = q(:,1);
% con_y = q(:,2);
% con_theta = q(:,3);
% con_psi = q(:,4);
% con_phi = q(:,5);

%q = q(N:num_samples+N,:);

con_x = q(:,1);
con_y = q(:,2);
con_theta = q(:,3);
con_psi = q(:,4);
con_phi = q(:,5);

[con_x_dot, ~] = firstsecondderivatives(t_aux(1:length(q)),con_x);
[con_y_dot, ~] = firstsecondderivatives(t_aux(1:length(q)),con_y);

%% Wrapping

angle_diff = mod(wrapTo2Pi(x0(3)) - wrapTo2Pi(con_theta(1)) + pi, 2*pi) - pi; % signed angle difference

realFirst = x0(3)-angle_diff;
con_theta = con_theta-con_theta(1);
con_theta = unwrap(con_theta+realFirst);

if model == "standardrwdtractortrailer"

    xR_vec = con_x - L*cos(con_theta) - d*cos(con_theta+con_phi);
    yR_vec = con_y - L*sin(con_theta) - d*sin(con_theta+con_phi);

    for i = 1:length(q)

        [~,index] = nearestPoint(xR_vec,yR_vec,con_x(i),con_y(i));

        x_final(i) = con_x(index);
        y_final(i) = con_y(index);
        theta_final(i) = con_theta(index);
        psi_final(i) = con_psi(index);
        phi_final(i) = con_phi(index);
        x_dot_final(i) = con_x_dot(index);
        y_dot_final(i) = con_y_dot(index);

    end

else

    for i = 1:length(q)

        x_final(i) = con_x(i);
        y_final(i) = con_y(i);
        theta_final(i) = con_theta(i);
        psi_final(i) = con_psi(i);
        phi_final(i) = con_phi(i);
        x_dot_final(i) = con_x_dot(i);
        y_dot_final(i) = con_y_dot(i);

    end
end

q_aux = [x_final',y_final',theta_final',psi_final',phi_final',x_dot_final',y_dot_final'];

for i=1:N-1
    tail_configuration(i,:) = q_aux(end,:);
end

q_aux = [q_aux(N:end,:); tail_configuration];
%x_opt = q_aux';


%% Plot trajectory and optimal control

% figh = figure(50);
% set(gcf, 'Color', 'w');
% name = strcat(motion,'_RRT_',model,'_IOfeedbackLinearization');
% set(gcf, 'Name', name);
%
% hold on
% plot(x_frwd(end-N+1:end),y_frwd(end-N+1:end),'b','linewidth',1); hold on
% plot(x_opt(1,:),x_opt(2,:),'r--','linewidth',1.5)
% axis square, axis equal, grid on,... %xlim('padded'),ylim('padded'),...
%     xlabel('$$x \, [m]$$','Interpreter','latex'), ylabel('$$y \, [m]$$','Interpreter','latex'),...
%     %title('I-O feedback linearization','Interpreter','latex')
%
% indices = round(linspace(1, length(x_opt), 2)); % select n equally spaced indices
% %indices = [1,length(x_opt)]; % select n equally spaced indices
% k = 1;
%
% for i = 1:length(x_opt)
%
%     if ismember(i, indices(1:end)) % Check if the current index is in the array
%
%         switch model
%
%             case "standardrwdtractortrailer"
%                 % Standard tractor-trailer model
%                 xR = x_opt(1,i);
%                 yR = x_opt(2,i);
%                 theta = x_opt(3,i);
%                 psi = x_opt(4,i);
%                 phi = x_opt(5,i);
%
%             case "pointp_rwdtractortrailer"
%                 % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
%                 % x = xR + L*cos(theta) + d*cos(theta+phi)
%                 % y = yR + L*sin(theta) + d*sin(theta+phi)
%
%                 xR = x_opt(1,i) - L*cos(x_opt(3,i)) - d*cos(x_opt(3,i)+x_opt(5,i));
%                 yR = x_opt(2,i) - L*sin(x_opt(3,i)) - d*sin(x_opt(3,i)+x_opt(5,i));
%                 theta = x_opt(3,i);
%                 psi = x_opt(4,i);
%                 phi = x_opt(5,i);
%
%             otherwise
%                 error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
%         end
%
%         xF = xR+L*cos(theta);
%         yF = yR+L*sin(theta);
%
%         hitchJoint = [xR - L1*cos(theta),yR - L1*sin(theta)];
%         xTrailer = hitchJoint(1) - L2*cos(theta+psi);
%         yTrailer = hitchJoint(2) - L2*sin(theta+psi);
%
%             plotArrow = 1;
%
%         plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
%         plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
%         r = wheelDiam/3;
%         c1 = wheelRGB;
%         c2 = "r";
%         circles = plotFilledCircle(x_opt(1,i),x_opt(2,i),r,c1,c2);
% %         labels(k) = 'P' + " "  + num2str(k);
% %         labelpoints(x_opt(1,i),x_opt(2,i),labels(k),'interpreter','latex','stacked', 'up','buffer',1.5);
% %         k = k+1;
%         labels(k) = "t = " + num2str(t_aux(i)) + " [s]";
%         labelpoints(xTrailer,yTrailer,labels(k),'interpreter','latex','S',1.5, 'adjust_axes',1);
%         k = k+1;
%     end
% end
%
% if model == "standardrwdtractortrailer"
%     xR_vec = con_x - L*cos(con_theta) - d*cos(con_theta+con_phi);
%     yR_vec = con_y - L*sin(con_theta) - d*sin(con_theta+con_phi);
%
% [~,index] = nearestPoint(xR_vec,yR_vec,con_x(end),con_y(end));
%
% theta_final = con_theta(index);
% psi_final = con_psi(index);
% phi_final = con_phi(index);
%
%         xR = xR_vec(index);
%         yR = yR_vec(index);
%         xF = xR+L*cos(theta_final);
%         yF = yR+L*sin(theta_final);
%
%         hitchJoint = [xR - L1*cos(theta_final),yR - L1*sin(theta_final)];
%         xTrailer = hitchJoint(1) - L2*cos(theta_final+psi_final);
%         yTrailer = hitchJoint(2) - L2*sin(theta_final+psi_final);
%
%         plotArrow = 1;
%         plot_tractor(xR,xF,yR,yF,hitchJoint,theta_final,phi_final,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow);
%         plot_trailer(xTrailer,yTrailer,hitchJoint,theta_final,psi_final,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow);
% end
%
% axis padded
%legend
%legend(["Reference trajectory","Auxiliary trajectory",tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer')],'Interpreter','latex','Orientation','horizontal','Location','bestoutside')

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function nc = noCollision(p2, p1, obs)

% check if line segment intersects polygon
is_intersecting = false;
for j = 1:length(obs)
    % polygon vertices
    x = obs(j).coord(:,1);
    y = obs(j).coord(:,2);
    x = [x; x(1)];
    y = [y; y(1)];

    % check if line segment intersects polygon

    [xi, yi] = intersections([p1(1) p2(1)],[p1(2) p2(2)],x,y,0);

    if ~isempty(xi) && ~isempty(yi)
        is_intersecting = true;
        break;
    end

end

if is_intersecting==false
    nc=1;

else
    nc=0;
end

end

function A = steer(qr, qn, val, eps)

% The code first computes the distance between qr and qn using the dist
% function. The distance is used to calculate the fraction of the distance
% that can be covered in a single step of size eps. This is done by
% computing the ratio of eps to the distance between qr and qn. This
% fraction is multiplied with the x and y components of the vector pointing
% from qn to qr. This gives the x and y components of a new vector that
% points towards qr with a magnitude of eps.
%
% Finally, the x and y components of this new vector are added to the x and
% y coordinates of qn respectively, giving the new point qnew that is one
% step closer to the target point qr.

qnew = [0 0];

% Steer towards qn with maximum step size of eps
if val >= eps
    qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
    qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
else
    qnew(1) = qr(1);
    qnew(2) = qr(2);
end
A = [qnew(1), qnew(2)];
end

function d = dist(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
end