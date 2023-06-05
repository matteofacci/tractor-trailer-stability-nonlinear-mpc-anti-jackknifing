function dq = shift_model(t, q, input, model)
    
    global L L1 L2 d
    
    theta = q(3);
    psi = q(4);
    phi = q(5);

    v = input(1);
    omega = input(2);

switch model

    case "standardrwdtractortrailer"
        % Standard tractor-trailer model
        dq = [v*cos(theta);
            v*sin(theta);
            v*tan(phi)/L;
            -(v*tan(phi)/L)*((L1/L2)*cos(psi)+1)-(v/L2)*sin(psi);
            omega];

    case "pointp_rwdtractortrailer"
        % mi permette di non entrare in singolarità
        % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
        % x = xR + L*cos(theta) + d*cos(theta+phi)
        % y = yR + L*sin(theta) + d*sin(theta+phi)

        dq = [v*(cos(theta) - (L*sin(theta) + d*sin(theta+phi))*tan(phi)/L) - d*sin(theta+phi)*omega;
            v*(sin(theta) + (L*cos(theta)+ d*cos(theta+phi))*tan(phi)/L) + d*cos(theta+phi)*omega;
            v*tan(phi)/L;
            -(v*tan(phi)/L)*((L1/L2)*cos(psi)+1)-(v/L2)*sin(psi);
            omega];

    otherwise
        error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
end