% This script is using the CasADi toolbox for nonlinear optimization. It is
% solving a nonlinear model predictive control (NMPC) problem for a robot
% with a defined prediction horizon N and time step T. The robot's states
% (x,y,theta,psi,phi) and controls (v,omega) are defined as symbolic
% variables, and the system's right-hand side is defined using these
% variables. The objective function of the MPC problem is defined as a
% weighted sum of the difference between the predicted states and a
% reference trajectory, as well as the control inputs.The script is using
% IPOPT, a nonlinear programming solver, to solve the problem.

clc; clear all; close all; warning off;
% Change the current folder to the folder of this m-file.
if(~isdeployed)
    cd(fileparts(matlab.desktop.editor.getActiveFilename));
end
% delete(findall(0)); % For forcing the closure of UI figures

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));
addpath(genpath('videos'));
addpath(genpath('figures'));
addpath(genpath('workspaces'));

global N T

% Import CasADi
import casadi.*

saveFigs = 0; % Set 1 to save all the opened figures
import_workspace = 0; % Set 1 to import a workspace and skip simulation
format = 'eps';
% PDF: pdf
% EPS: eps
% JPEG: jpeg
% TIFF: tiff
% PNG: png

% Define the final duration of the video
finalDuration = 30; % [s] set 0 for the real simulation time

%% Simulation parameters

T = 0.1; % [s] time step
MPC_control_horizon = 5; % [s]
N = floor(MPC_control_horizon/T); % control horizon, number of steps to look ahead for optimization
sim_time = 20; % [s] total time for the simulation

new_range_N = N*2; % augmented N for IO linearization

%% Desired trajectory type

%motion = "forward";
motion = "backward";
% motion = "complete";

RRT = 0; % Set 1 to create random obstacles and consequent trajectory [15s] CH

% Input trajectory type: "Linear", "Circular", "Lemniscate"
trajectory = "Linear"; %[25s] sim_time and [10]s horizon for a complete trajectory
%trajectory = "Circular"; % [32s] sim_time and [5]s horizon for a complete trajectory
%trajectory = "Lemniscate"; % [220s] sim_time and [14]s horizon for a complete trajectory
%trajectory = "Rectangular"; % [100s] sim_time [15]s horizon (HANDLE WITH CARE!!!)

trajectory = lower(trajectory); % convert the input to lower case for uniformity

%terminal_constraint = "basic";
terminal_constraint = "advanced";

%output_error_FL = "online"; % it works only with horizon > 5s (HANDLE WITH CARE!!!)
output_error_FL = "offline";

t0 = 0; % starting instant
%x0 = [0 ; 1.25 ; pi ; 0.0 ; 0.0]; % initial condition

x0 = initialAlignment(trajectory,motion,t0,N,T);

%x0 = [1.25,0,pi/2,0,0]'; % constraints circular
%x0 = [0,-1,5*pi/4,0,0]'; % constraints lemniscate
%x0 = [1.75,0,pi/4,0,0]'; % constraints shorter ch circular
%x0 = [0,0,5*pi/4,0,0]'; % constraints shorter ch leminsicate
%x0 = [6,0.01,0,0,0]'; % linear paper
%x0 = [4.99,5,1.5,0.03,0.05]'; % circular paper
%x0 = [0,0.05,3.92,-0.09,0]'; % lemnsicate paper
%x0(3) = -pi/2;
%x0 = [6,0.5,-pi/4,0,0]'; % linear paper
x0 = [6,0.5,-pi/4,0,0]'; % linear paper
%x0 = x0+ek;
x0(3) = wrapTo2Pi(x0(3));

%ek = [0;0.5;-0.1;0.5;0];

% x0(1) = x0(1)+0.5;
% x0(3) = x0(3)-pi/4;
%x0(4) = x0(4)+0.03;
%
% x0(2) = x0(2)+0.5;
% x0(3) = x0(3)-pi/4;

% x0(3) = -0.1;
% x0(4) = 0.5;
% 
% x0(3) = wrapTo2Pi(x0(3));

% x0 = [0,0.05,3.92,-0.09,0]'; % paper lemniscate
% % %x0 = [x0(1)-0.01,x0(2),-0.07+pi/2, 0.03, 0.05]'; % paper circular
% x0(3) = wrapTo2Pi(x0(3));


%% Model

%model = "standardRWDTractorTrailer"; % select for controlling the rear axle (HANDLE WITH CARE!!!)
model = "pointP_RWDTractorTrailer"; % select for controlling a point P at distance L+d from the rear axle

model = lower(model); % convert the input to lower case for uniformity
tractorTrailerModel;

f = Function('f', {states, controls}, {xdot}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_c,N); % decision variables for the control inputs

if motion == "backward"

    tot_stability_cons = 3; % total amount of the stability constraints (one for each angle)

    P = SX.sym('P',n_s + N*(n_s+n_c)+n_s+tot_stability_cons*n_s); % parameters (which include the initial state and the reference state)

else
    P = SX.sym('P',n_s + N*(n_s+n_c)+n_s); % parameters (which include the initial state and the reference state)
end

X = SX.sym('X',n_s,(N+1)); % vector that represents the states over the optimization problem

%% I-O linearization parameters
K1 = 1;
K2 = 1;

%% RRT algorithm

if RRT
    output_error_FL = "offline";

    IO_linearization_tic = tic;

    [q_ref,final_constraint,x0,obs,obs_reduced,N_obs,sim_time,v_aux,omega_aux] = RRTstar(motion,model,n_s,K1,K2);

    for i=1:new_range_N
        tail_configuration(i,:) = q_ref(end,:);
    end

    ref_temp = [q_ref; tail_configuration];

    x_aux_final = final_constraint(:,1);
    y_aux_final = final_constraint(:,2);
    theta_aux_final = final_constraint(:,3);
    psi_aux_final = final_constraint(:,4);
    phi_aux_final = final_constraint(:,5);
    x_dot_aux_final = final_constraint(:,6);
    y_dot_aux_final = final_constraint(:,7);

    IO_linearization_toc = toc(IO_linearization_tic);
end

%% Objective function, Runge-Kutta model and multiple shooting constraints

% The objective function is calculated by summing up the state-parameter
% and input-parameter weighted by Q and R matrices respectively.

obj = 0; % initialize the objective function
g = []; % initialize the constraints vector

% Weight matrix (states)
Q = zeros(5,5);
% Q(1,1) = 100000;
% Q(2,2) = 100000;
Q(1,1) = 1000;
Q(2,2) = 1000;
Q(3,3) = 0;
Q(4,4) = 0;
Q(5,5) = 0;

% Weight matrix (controls)
R = zeros(2,2);
R(1,1) = 10;
R(2,2) = 100;

% Weight matrix (states)
V = zeros(5,5);
% Q(1,1) = 100000;
% Q(2,2) = 100000;
% V(1,1) = 1;
% V(2,2) = 1;
% V(3,3) = 0;
% V(4,4) = 0;
% V(5,5) = 0;

% gamma = 0.5; delta = 1; epsilon = 1; eta = 1; % weighting parameters

gamma = 1000;
%delta = 10;

delta = zeros(2,2);
delta(1,1) = 1000;
delta(2,2) = 10;

state  = X(:,1); % get the initial state from the X vector
g = [g;state-P(1:n_s)]; % add the initial condition as a constraint
%constraintVec = []; % initialize the constraint vector

for k = 1:N % for loop to iterate through the prediction horizon
    state = X(:,k);  input = U(:,k); % get the current state and control input

    if k == 1 %|| RRT
        sum5 = 0;
        %input_prev = U(:,k);
    else
        input_prev = U(:,k-1);
    %end

        % u_temp = [input_prev,input];
        % 
        % %du_temp = [diff(u_temp(1,:));diff(u_temp(2,:))];
        % 
        % v_norm = norm(u_temp(1,:));
        % omega_norm = norm(u_temp(2,:));
        % 
        % u_norm = [v_norm;omega_norm];
        % sum5 = u_norm'*delta*u_norm;
        % %sum5 = du_temp'*delta*du_temp;
        % 
        % % Calculate the distance between input and input_prev
        % input_dist(1) = norm(input - input_prev)^2;

        sum5 = (input - input_prev)'*delta*(input - input_prev);

    end



    % Breaking down the objective function calculation into individual terms
    sum1=(state-P(7*k-1:7*k+3))'*Q*(state-P(7*k-1:7*k+3)); % calculate the state-parameter term

    % if motion == "backward" && output_error_FL == "offline" && not(RRT)
    %sum2=(input-P(7*k+4:7*k+5))'*R*(input-P(7*k+4:7*k+5)); % calculate the input-parameter term
    % else
    sum2 = (input)'*R*(input); % +delta*abs(input(1))+delta*abs(input(2));
    %end
    %    sum3 = delta*diff(input,2);
    %    sum4=delta*abs(input(1))+delta*abs(input(2)); % smoothing input
    %     input_prev = [P(end-1),P(end)]';
    % %sum5 = delta*(P(end-N*2+k)*input(1))+delta*(P(end-N+k)*input(2));
    % %sum5 = delta*(P(end-N*2+k)*input(1))+delta*(P(end-N+k)*input(2));
    % %sum5 = (input_prev(1)*input(1))'*delta*(input_prev(1)*input(1));
    %
    % if k == 1
    %     sum5 = (input_prev(1)-input(1))'*delta*(input_prev(1)-input(1));
    % else
    %     sum5 = 0;
    % end

    %input_prev = [P(end-1),P(end)]';
    %sum5 = delta*(P(end-N*2+k)*input(1))+delta*(P(end-N+k)*input(2));
    %sum5 = delta*(P(end-N*2+k)*input(1))+delta*(P(end-N+k)*input(2));
    %sum5 = (input_prev(1)*input(1))'*delta*(input_prev(1)*input(1));
    %sum5 = (input_prev(1)*input(1))'*delta*(input_prev(1)*input(1));


    obj=obj + T*(sum1+sum2+sum5); %+sum3); %+sum4);
    %sum3+sum4+sum5+sum6);

end
%    obj = obj + (X(:,end)-P(end-n_s+1:end))'*V*(X(:,end)-P(end-n_s+1:end)); % final state term
obj = obj + (X(:,end)-P(n_s + N*(n_s+n_c)+1:n_s + N*(n_s+n_c)+n_s))'*V*(X(:,end)-P(n_s + N*(n_s+n_c)+1:n_s + N*(n_s+n_c)+n_s)); % final state term

for k = 1:N % for loop to iterate through the prediction horizon
    state = X(:,k);  input = U(:,k); % get the current state and control input

    % Runge-Kutta 4th order integration to calculate the state_next and
    % then use this to compute the constraints for the current timestep.
    state_next = X(:,k+1);
    k1 = f(state, input); % new state using the nonlinear model
    k2 = f(state + 0.5*T*k1, input); % new state using the nonlinear model
    k3 = f(state + 0.5*T*k2, input); % new state using the nonlinear model
    k4 = f(state + T*k3, input); % new state using the nonlinear model
    state_next_RK4=state +T/6*(k1 +2*k2 +2*k3 +k4); % new state using Runge Kutta 4th order integration

    g = [g;state_next-state_next_RK4]; % compute constraints (multiple-shooting)
end

%% Stability constraint

if motion == "backward"

    %         T_u = [0,0,1,0,0;
    %             0,0,0,1,0;
    %             0,0,0,0,1];

    T_u = reshape(P(end-(tot_stability_cons*n_s)+1:end),[3,5]); % transformation matrix


    g = [g; T_u*X(:,end-1)]; % IO linearization constraints
end


%% Simulation parameters and constraints on state and control variables

OPT_variables = [reshape(X,5*(N+1),1);reshape(U,2*N,1)]; % reshape the states and controls into one column vector for the decision variables

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P); % create a struct that contains the objective function, decision variables, constraints, and parameters of the problem

opts = struct; % create an empty struct to store options for the solver
opts.ipopt.max_iter = 1000; % maximum number of iterations for the solver
opts.ipopt.print_level = 0; % level of printing during the optimization, 0 for no printing, 3 for detailed printing
opts.print_time = 0; % do not print the time taken by the solver
opts.ipopt.acceptable_tol = 1e-4; % tolerance for the solver
opts.ipopt.acceptable_obj_change_tol = 1e-3; % tolerance for the change in the objective function
opts.expand = true;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts); % create an NLPSOL object to solve the problem using the IPOPT solver

args = struct; % create an empty struct to store the bounds and constraints

% Equality constraints
args.lbg(1:n_s*(N+1)) = 0; % lower bound on the equality constraints
args.ubg(1:n_s*(N+1)) = 0; % upper bound on the equality constraints

% State variable bounds
args.lbx(1:n_s:n_s*(N+1)) = -inf; % lower bound on the x state variable
args.ubx(1:n_s:n_s*(N+1)) = inf; % upper bound on the x state variable
args.lbx(2:n_s:n_s*(N+1)) = -inf; % lower bound on the y state variable
args.ubx(2:n_s:n_s*(N+1)) = inf; % upper bound on the y state variable
args.lbx(3:n_s:n_s*(N+1)) = -inf; % lower bound on the theta state variable
args.ubx(3:n_s:n_s*(N+1)) = inf; % upper bound on the theta state variable
args.lbx(4:n_s:n_s*(N+1)) = min_hitch; % lower bound on the psi state variable
args.ubx(4:n_s:n_s*(N+1)) = max_hitch; % upper bound on the psi state variable
args.lbx(5:n_s:n_s*(N+1)) = min_steering; % lower bound on the phi state variable
args.ubx(5:n_s:n_s*(N+1)) = max_steering; % upper bound on the phi state variable

% Control input bounds
args.lbx(n_s*(N+1)+1:2:n_s*(N+1)+2*N) = v_min; % lower bound on the v control input
args.ubx(n_s*(N+1)+1:2:n_s*(N+1)+2*N) = v_max; % upper bound on the v control input
args.lbx(n_s*(N+1)+2:2:n_s*(N+1)+2*N) = omega_min; % lower bound on the omega control input
args.ubx(n_s*(N+1)+2:2:n_s*(N+1)+2*N) = omega_max; % upper bound on the omega control input

%----------------------------------------------
% END OF PROBLEM SET UP

%% START THE SIMULATION
%-------------------------------------------

%% Initialize the simulation

x_opt(:,1) = x0; % x_opt contains the history of states

t(1) = t0;

u0 = zeros(N,2); % two control inputs for each robot
%u_prev = [v_min,0]; % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

mpciter = 0; % initialize the mpc iteration counter
x_pred = []; % x_pred contains the predicted states
final_constraint = [];
u_opt=[]; % initialize the control input history


%% Offline Output Error Feedback Linearization

if motion == "backward" && output_error_FL == "offline" && not(RRT)

    IO_linearization_tic = tic;

    [x_aux_final, y_aux_final, theta_aux_final, psi_aux_final, phi_aux_final, x_dot_aux_final, y_dot_aux_final,v_aux,omega_aux] = auxiliaryTrajectory_offline(t0,floor(sim_time/T)+new_range_N,x0,motion,model,trajectory,n_s,K1,K2);

    IO_linearization_toc = toc(IO_linearization_tic);

    final_constraint = [x_aux_final', y_aux_final', theta_aux_final', psi_aux_final', phi_aux_final', x_dot_aux_final', y_dot_aux_final'];
    %feedfrwd = [v_aux;omega_aux]';

    %feedfrwd = mean(v_aux(1:floor(sim_time / T)));
end

%% Start MPC

main_loop = tic; % start timing the main loop
reverseStr = ''; % string for the command window fprintf during the simulation
flag_tolerance = 0;
% progressbar = waitbar(0,sprintf('Processed %d/%d samples', mpciter+1, sim_time/T),'Name','Simulating MPC process...',...
%     'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
%
% setappdata(progressbar,'canceling',0);


% These lines of code are the main simulation loop that runs the MPC
% algorithm. The loop runs as long as the number of MPC iterations is less
% than the maximum simulation time divided by the time step. Within the
% loop, the current time is calculated and the initial condition of the
% robot posture is set as the parameter for the optimization problem. The
% reference trajectory to be tracked is also set within the loop. The loop
% uses the nlpsol function to solve the optimization problem and update the
% control inputs. The main_loop variable is used to time the execution of
% the loop.

while (mpciter < floor(sim_time / T)) % condition for ending the loop
    current_time = t(1)+mpciter*T;  % get the current time

    %args.p(1:5) = x0; % initial condition of the robot posture
    reference = [];

    % Create reference trajectory
    if RRT
        reference = ref_temp(mpciter+1:mpciter+N+1,:);
    else
        for k = 1:new_range_N
            [t_predict,x_ref,y_ref,theta_ref,psi_ref,phi_ref,u_ref,omega_ref] = referenceTrajectory(current_time,k,T,motion,trajectory);

            if motion == "backward"

                % Online Output Error Feedback Linearization
                if output_error_FL == "offline"

                    u_ref = v_aux(k);
                    omega_ref = omega_aux(k);
                end
            end

            if k <= N+1
                reference = [reference;x_ref,y_ref,theta_ref,psi_ref,phi_ref,u_ref,omega_ref];
            end

            % Auxiliary variables for the IO feedback linearization
            x_aux(k) = x_ref;
            y_aux(k) = y_ref;
            t_aux(k) = t_predict;
        end
    end

    final_state_ref = reference(end,1:n_s);
    %final_state_ref = [x_final,y_final,theta_final,psi_final,phi_final];
    reference = reference(1:end-1,:);

    % Set the reference to track in the parameters vector
    args.p = [x0', reshape(reference.', [], 1)']; % reshape the reference into a 1D vector
    args.p = [args.p,final_state_ref];

    %     for k=1:N % set the reference to track in the parameters vector
    %         args.p(7*k-1:7*k+3) = [reference(k,1), reference(k,2), reference(k,3),reference(k,4),reference(k,5)];
    %         args.p(7*k+4:7*k+5) = [reference(k,6), reference(k,7)];
    %     end

    if motion == "backward"

        % Online Output Error Feedback Linearization
        if output_error_FL == "online"

            IO_linearization_tic = tic;

            [x_final,y_final,theta_final,psi_final,phi_final,x_dot_final,y_dot_final] = auxiliaryTrajectory_online(motion,model,x0,t_aux,x_aux,y_aux,n_s,K1,K2);

            IO_linearization_toc(mpciter+1) = toc(IO_linearization_tic);

            final_constraint = [final_constraint;x_final,y_final,theta_final,psi_final,phi_final];
        else
            x_final = x_aux_final(mpciter+1);
            y_final = y_aux_final(mpciter+1);
            x_dot_final = x_dot_aux_final(mpciter+1);
            y_dot_final = y_dot_aux_final(mpciter+1);
            theta_final = theta_aux_final(mpciter+1);
            psi_final = psi_aux_final(mpciter+1);
            phi_final = phi_aux_final(mpciter+1);
        end


        if terminal_constraint == "advanced"
            ak = matrixA(x_dot_final,y_dot_final,theta_final,psi_final,phi_final,K1,K2);
            A(:,:,mpciter+1) = ak;

            [V,D] = eig(ak);
            Vb = [V(:,4)/V(1,4), V(:,5)/V(2,5), V(:,1), V(:,3), V(:,2)];

            trans_matrix(:,:,mpciter+1) = inv(Vb);


            % The stability matrix is the same of T_u, used as numerical
            % matrix and not as parametric matrix as T_u

            stability_matrix = trans_matrix(3:end,:,mpciter+1);

        else

            stability_matrix = [0,0,1,0,0; 0,0,0,1,0; 0,0,0,0,1];

        end

        args.p = [args.p,reshape(stability_matrix, [], 1)'];

        args.lbg(n_s*(N+1)+1:n_s*(N+1)+tot_stability_cons) = stability_matrix*[x_final,y_final,theta_final,psi_final,phi_final]'; % lower bound on the equality constraints
        args.ubg(n_s*(N+1)+1:n_s*(N+1)+tot_stability_cons) = stability_matrix*[x_final,y_final,theta_final,psi_final,phi_final]'; % upper bound on the equality constraints

    end

    % Feedforward with v and omega aux

    if motion == "forward"
        u0(:,1) = v_max*ones(N,1); % Feedforward
    else
        % if RRT
        u0(:,1) = v_min*ones(N,1); % Feedforward
        % else
        %u0 = u0+[v_aux(mpciter+1),omega_aux(mpciter+1)].*ones(N,2);
        %u0 = u0+[v_min,omega_aux(mpciter+1)].*ones(N,2);
        %u0(:,1) = v_min*ones(N,1); % Feedforward
        %     end
    end

    args.x0 = [reshape(X0',5*(N+1),1);reshape((u0+[v_aux(mpciter+1),omega_aux(mpciter+1)].*ones(N,2))',2*N,1)];
    %args.x0 = [reshape(X0',5*(N+1),1);reshape(u0',2*N,1)]; % set the initial value of the decision variables
    %args.p = [args.p,reshape(u_prev',2*N,1)'];
    %args.p = [args.p,u_prev];
    solver_tic = tic;

    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p); % use the 'solver' function to solve the optimization problem, with the initial values, bounds, and parameters specified in 'args' structure

    solver_toc(mpciter+1) = toc(solver_tic);

    % This following code processes the solution obtained from the optimization
    % problem and uses it to update the current state and control inputs
    % for the next iteration of the simulation loop.

    u = reshape(full(sol.x(5*(N+1)+1:end))',2,N)'; % reshape the part of the solution vector that contains the control inputs, so it can be easily used

    x_pred(:,1:5,mpciter+1)= reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % reshapes the part of the solution vector that contains the predicted states, so it can be easily used

    %u_prev = u(1,:);
    u_opt = [u_opt ; u(1,:)]; % take just the first control from the solution and appends it to the "u_opt" matrix

    t(mpciter+1) = t0; % store the current time.

    [t0, x0, u0] = shift(T, t0, x0, u,f,model); % apply the first control input to the system and update the state and time for the next iteration

    %u0(:,1) = u0(:,1)+feedfrwd(mpciter+1)*ones(N,1);

    x_opt(:,mpciter+2) = x0; % store the current state

    position_error(mpciter+1,:) = sqrt((reference(1,1) - x_opt(1,mpciter+1))^2 + (reference(1,2) - x_opt(2,mpciter+1))^2);

    X0 = reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % reshape the part of the solution vector that contains the solution trajectory so it can be easily used

    X0 = [X0(2:end,:);X0(end,:)]; % shift the solution trajectory so it can be used as the initial condition for the next iteration

    msg = sprintf('Processed: %d samples (Total: %d) - [%.2f/100]', mpciter+1, floor(sim_time / T),(mpciter+1)*100/(floor(sim_time / T))); % display the current iteration
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));

    % waitbar(mpciter/(sim_time/T),progressbar,sprintf('Processed %d/%d samples', mpciter+1, sim_time/T));

    loop_time = toc(main_loop); % store the total time taken by the loop
    average_partial_time = loop_time/(mpciter+1); % calculate the average time taken for each MPC iteration

    % if (position_error(mpciter+1) > 0.8) || average_partial_time > 3
    %     fprintf('\nFailure: Maximum tolerance reached.')
    %     flag_tolerance = 1;
    %     break
    % end

    mpciter = mpciter + 1; % increment the iteration counter

end

%u_opt(:,1) = moving_average(2,u_opt(:,1));
%u_opt(:,2) = moving_average(2,u_opt(:,1));

%delete(progressbar)
main_loop_time = toc(main_loop); % store the total time taken by the loop
average_main_loop_time = main_loop_time/(mpciter+1); % calculate the average time taken for each MPC iteration

if motion == "backward" && output_error_FL == "online"
    average_IO_linearization_time = sum(IO_linearization_toc)/(mpciter+1);
else
    average_IO_linearization_time = 0;
end
average_solver_time = sum(solver_toc)/(mpciter+1);

starting_time = t(1);

if RRT
    x_ref = q_ref(:,1)';
    y_ref = q_ref(:,2)';
else

    for k = 1:length(t)
        [~,x_temp,y_temp,~,~,~,~,~] = referenceTrajectory(starting_time,k,T,motion,trajectory);
        x_ref(k) = x_temp;
        y_ref(k) = y_temp;
    end
end

% Truncate vectors to the same length
max_length = min(length(t), length(x_opt(3,:)));
t = t(1:max_length);
t_ref = t;
x_ref = x_ref(1:max_length);
y_ref = y_ref(1:max_length);
xx_temp(:,:) = x_opt(:,1:max_length);
x_opt = xx_temp;

% Position RMSE

rmse_x = sqrt(mean((x_ref - x_opt(1,:)).^2)); % Root Mean Squared Error
rmse_y = sqrt(mean((y_ref - x_opt(2,:)).^2)); % Root Mean Squared Error
position_rmse = sqrt(rmse_x^2+rmse_y^2); % Root Mean Squared Error

% Max error

max_error = max(position_error);

% Settling time

threshold = 0.01;

index = find(position_error >= threshold, 1, 'last');

if index
    for i=1:length(t)
        if i >= index && position_error(i) < threshold
            settling_time = t(i);
            break
        else
            settling_time = nan;
        end
    end
else
    settling_time = nan;
end

% Final prints

fprintf('\nAverage time for each algorithm iteration: %.2f [ms]\n', average_main_loop_time*1000)
fprintf('Average time for each MPC iteration: %.2f [ms]\n', (average_main_loop_time-average_IO_linearization_time)*1000)
fprintf('Average time for each solver operation: %.2f [ms]\n', average_solver_time*1000)

if motion == "backward" && output_error_FL == "online"
    fprintf('Average time for each OEFL: %.2f [ms]\n', (average_IO_linearization_time)*1000)
elseif motion == "backward" && output_error_FL == "offline"
    fprintf('Total time for OEFL: %.2f [ms]\n', (IO_linearization_toc)*1000)
end

fprintf('Settling time: %.2f [s]\n', settling_time)
fprintf('Position RMSE: %.2f [m]\n', position_rmse)
fprintf('Peak cartesian error: %.2f [m]\n', max_error)

%% Plots and videos

if flag_tolerance == 0

    plotsFlag = string_from_user('\nShow plots: Y/N [N]: ','N');
    animationFlag = string_from_user('Show animation: Y/N [N]: ','N');
    renderVideoFlag = string_from_user('Save final video: Y/N [N]: ','N');
    workSpaceFlag = string_from_user('Save workspace: Y/N [N]: ','N');

    if or(workSpaceFlag == 'Y', workSpaceFlag == 'y')
        saveWorkspace({'motion','trajectory','sim_time','MPC_control_horizon'})
    end

    if or(animationFlag == 'Y', animationFlag == 'y')
        drawAnimation;
    end

    if or(plotsFlag == 'Y', plotsFlag == 'y')
        plots;
    end

    if or(renderVideoFlag == 'Y', renderVideoFlag == 'y')
        renderVideo;
    end

end
