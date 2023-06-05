function [t_predict,x_ref,y_ref,theta_ref,psi_ref,psi2_ref,phi_ref,u_ref,omega_ref] = referenceTrajectory(current_time,k,T,motion,trajectory)

t_predict = current_time + (k-1)*T; % predicted time instant

switch trajectory

    case "linear"
        % code for generating a linear trajectory

        x_ref = 0.25*t_predict;
        y_ref = 0;
        theta_ref = 0;
        % Compute x_dot and y_dot for linear trajectory
        x_dot = 0.25;
        y_dot = 0;

    case "circular"
        % code for generating a circular trajectory

        %% Circular Trajectory (Forward)

        % to complete one full circle in 10 seconds, the frequency would be 0.1 Hz and the angular velocity would be:
        % ang_vel = (2*pi*0.1) = 0.6283 rad/s

        r = 1.25;
        x_center = 0;
        y_center = 0;
        v = 0.2;
        ang_vel = v / r;
        %ang_vel = -1/5;

        x_ref = x_center + r*cos(ang_vel* t_predict);
        y_ref = y_center + r*sin(ang_vel* t_predict);
        % theta_ref = atan2(y_ref,x_ref);
        theta_ref = 0;

        % Compute x_dot and y_dot for circular trajectory
        x_dot = -r*sin(ang_vel*t_predict);
        y_dot = r*cos(ang_vel*t_predict);


    case "lemniscate"

        % The lemniscate trajectory is defined by the equation:
        % (x^2/a^2) + (y^2/b^2) = 1
        %
        % where x and y are the coordinates of the point on the trajectory, and a
        % and b are parameters that determine the size and shape of the lemniscate.
        %
        % In terms of the angular velocity (ω) and time (t), the coordinates of the
        % point on the lemniscate trajectory can be represented as: x =
        % acos(ωt)sqrt(cos(2ωt)) y = bsin(ωt)sqrt(cos(2ωt))
        %
        % The velocity of the point on the lemniscate trajectory can be represented
        % as: x_dot = -aωsin(ωt)sqrt(cos(2ωt)) - 2acos(ωt)cos(2ωt) y_dot =
        % bωcos(ωt)sqrt(cos(2ωt)) - 2bsin(ωt)cos(2ωt)
        %
        % Note that the sign of the angular velocity (ω) can be changed to make the
        % trajectory move forward or backward.
        % code for generating a Lemniscate trajectory

        a = 5;
        b = 5; %1.25
        v = 0.2;
        %ang_vel = v / (a/2);
        ang_vel = 1/35;

        x_ref = a*sin(ang_vel* t_predict);
        y_ref = b*sin(ang_vel*t_predict)*cos(ang_vel* t_predict);
        theta_ref = 0;

        % Compute x_dot and y_dot for lemniscate trajectory
        x_dot = a*ang_vel*cos(ang_vel*t_predict);
        y_dot = b*ang_vel*cos(2*ang_vel*t_predict);


    case "rectangular"

        % Define the coordinates of the four corners of the rectangle
        xA = 0;
        yA = 0;
        xB = 6;
        yB = yA;
        xC = xB;
        yC = 4;
        xD = xA;
        yD = yC;
        v = 0.25;

        % Calculate the total time to complete one lap around the rectangle
        lap_time = (xB-xA+yC-yA)*2/v;

        % Initialize variables to keep track of the current lap and t_predict
        current_lap = 0;

        current_lap = floor(t_predict/lap_time);

        t_lap = t_predict - current_lap * lap_time;

        % Determine which side of the rectangle the object is on at this value of t_predict
        if t_lap*v <= xB-xA
            % Object is on the bottom side of the rectangle
            x_ref = xA + t_lap*v;
            y_ref = yA;
            x_dot = v;
            y_dot = 0;
        elseif t_lap*v <= xB-xA+yC-yB
            % Object is on the right side of the rectangle
            x_ref = xB;
            y_ref = yA + (t_lap*v - (xB-xA));
            x_dot = 0;
            y_dot = v;
        elseif t_lap*v <= 2*(xB-xA)+yC-yA
            % Object is on the top side of the rectangle
            x_ref = xB - (t_lap*v - (xB-xA+yC-yB));
            y_ref = yC;
            x_dot = -v;
            y_dot = 0;
        elseif t_lap*v <= 2*(xB-xA)+2*(yC-yA)
            % Object is on the left side of the rectangle
            x_ref = xA;
            y_ref = - t_lap*v + 2*(xB-xA+yC-yA);
            x_dot = -v;
            y_dot = 0;
        end

        theta_ref = 0;

    otherwise
        error("Invalid trajectory type. Valid options are 'Linear', 'Circular', 'Lemniscate'")
end

psi_ref = 0;
psi2_ref = 0;
phi_ref = 0;

% Compute u_ref and omega_ref
u_ref = sqrt(x_dot^2 + y_dot^2);
omega_ref = (y_dot*x_dot)/(x_ref^2 + y_ref^2);

end