% This function is called to shift the state and input values, and the time
% forward by one step. The function takes in the time step T, the current
% time t0, the current state x0, the optimal control inputs u, and the
% dynamics function f.
% 
% It first assigns the first control value of the optimal solution to the
% input variable. Then it calculates the next state using the current
% state, input and dynamics function f_value. The next state is then
% assigned as the new initial state for the next iteration.
% 
% The current time is then advanced by T and the optimal control inputs are
% shifted to be used as initial guess for the next solution. The function
% returns the new time, state and input values.

function [t0, x0, u0] = shift(T, t0, x0, u,f,model)

state = x0;
input = u(1,:)'; % get the first control value of the optimal solution
%f_value = f(state,input); % propagate the control in the function f
[t,q] = ode45(@(t,q) shift_model(t,q,input,model),[0,T],state);
state = q(end,:)';
%state = state+ (T*f_value); % calculate the next state
x0 = full(state); % update the new initial state for the next iteration

t0 = t0 + T; % advance in time
u0 = [u(2:size(u,1),:);u(size(u,1),:)]; % the optimal control is used as initial guess for the next solution
% extracts all the rows from the second row to the last row of the matrix
% u, and all the columns of that matrix. u(size(u,1),:) extracts the last
% row of u and all the columns of that matrix.

end