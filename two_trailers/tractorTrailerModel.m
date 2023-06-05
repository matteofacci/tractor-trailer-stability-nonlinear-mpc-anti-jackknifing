global L L1 L2 L3 L4 d ...
    wheelRGB tractorRGB trailerRGB tractorBodyLenght trailerBodyLenght...
    tractorWidth trailerWidth wheelBase wheelWidth wheelDiam

% Import CasADi
import casadi.*

%% Model parameters

maximum_vel = 0.5; % [m/s] maximum linear velocity
minimum_vel = -maximum_vel; % minimum linear velocity

omega_max = 1.5; % [rad/s] maximum steering velocity
omega_min = -omega_max; % minimum steering velocity

max_hitch = pi/4; % upper bound on the psi state variable
min_hitch = -max_hitch; % upper bound on the psi state variable

max_steering = pi/12; % upper bound on the phi state variable
%max_steering = pi/6; % upper bound on the phi state variable
min_steering = -max_steering; % lower bound on the phi state variable

switch motion
    case "forward"
        v_max = maximum_vel; % maximum velocity
        v_min = 0 ; % minimum velocity
    case "backward"
        v_max = 0; % maximum velocity
        v_min = minimum_vel; % minimum velocity
    case "complete"
        v_max = maximum_vel; % maximum velocity
        v_min = minimum_vel; % minimum velocity
end

wheelRGB = [0,0,0]; %black
tractorRGB = 'r'; %red
trailerRGB = [0 0.4470 0.7410];

tractorBodyLenght = 0.45;
trailerBodyLenght = 0.367;
tractorWidth = 0.175;
trailerWidth = tractorWidth;
wheelBase = 0.14;
wheelWidth = (tractorWidth-wheelBase)/2;
wheelDiam = 0.045*2;

L = 0.255; L1 = 0.068; L2 = 0.262; L3 = 0.183; L4 = 0.262; d = 0.1; %tractorBodyLenght*0.5; % vehicle parameters

%% Casadi implementation

% State
x = SX.sym('x'); % x-coordinate of the vehicle's position
y = SX.sym('y'); % y-coordinate of the vehicle's position
theta = SX.sym('theta'); % yaw angle
psi = SX.sym('psi'); % hitch angle
psi2 = SX.sym('psi2'); % 2-nd hitch angle
phi = SX.sym('phi'); % front wheel angle

states = [x;y;theta;psi;psi2;phi]; % vector of all states
n_s = length(states); % number of state variables

% Control input
v = SX.sym('v'); % linear velocity
omega = SX.sym('omega'); % steering angle

controls = [v;omega]; % vector of all control inputs
n_c = length(controls); % number of control variables

tot_vars = n_s + n_c; % total number of variables (states + controls)

%% Nonlinear model of the vehicle

switch model

    case "standardrwdtractortrailer"
        % Standard tractor-trailer model
        xdot = [v*cos(theta);
            v*sin(theta);
            v*tan(phi)/L;
            -(v*tan(phi)/L)*((L1/L2)*cos(psi)+1)-(v/L2)*sin(psi);
            ((v*L1*tan(phi))/(L*L4))*((L2 + L3)*cos(psi)*cos(psi2)/L2 + L4*cos(psi)/L2 - ...
            cos(psi + psi2)) + (v*sin(psi)/L2)*((L2 + L3)*cos(psi2)/L4 + 1) - v*sin(psi + psi2)/L4;
            omega];

    case "pointp_rwdtractortrailer"
        % mi permette di non entrare in singolarità
        % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
        % x = xR + L*cos(theta) + d*cos(theta+phi)
        % y = yR + L*sin(theta) + d*sin(theta+phi)

        xdot = [v*(cos(theta) - (L*sin(theta) + d*sin(theta+phi))*tan(phi)/L) - d*sin(theta+phi)*omega;
            v*(sin(theta) + (L*cos(theta)+ d*cos(theta+phi))*tan(phi)/L) + d*cos(theta+phi)*omega;
            v*tan(phi)/L;
            -(v*tan(phi)/L)*((L1/L2)*cos(psi)+1)-(v/L2)*sin(psi);
            ((v*L1*tan(phi))/(L*L4))*((L2 + L3)*cos(psi)*cos(psi2)/L2 + L4*cos(psi)/L2 - ...
            cos(psi + psi2)) + (v*sin(psi)/L2)*((L2 + L3)*cos(psi2)/L4 + 1) - v*sin(psi + psi2)/L4;
            omega];

    otherwise
        error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
end