%% Draw/Render the Scenario

plotGhost = 0;

%figh = figureFullScreen(100);
figh = figure(1000);
set(gcf, 'Color', 'w');
%set(gcf,'Units','pixels','Position',[0 0 1280 720]);
screenSize = get(0, 'ScreenSize');
figWidth = 1280;
figHeight = 720;
% figWidth = 720;
% figHeight = 480;
figPos = [screenSize(3)/2 - figWidth/2, screenSize(4)/2 - figHeight/2, figWidth, figHeight];
set(gcf,'Units','pixels','Position', figPos);
txt = "MPC behavior   $$\rightarrow$$   Task duration : " + string(sim_time) + " [s]";

starting_time = t(1);

for k = 1:length(t)
    [~,x_temp,y_temp,~,~,~,~,~,~] = referenceTrajectory(starting_time,k,T,motion,trajectory);
    x_ref(k) = x_temp;
    y_ref(k) = y_temp;
end

% Truncate vectors to the same length
max_length = min(length(t), length(x_opt(3,:)));
t = t(1:max_length);
xx_temp(:,:) = x_opt(:,1:max_length);
x_opt = xx_temp;


for i = 1:size(x_opt,2)

    % Wipe the slate clean so we are plotting with a blank figure
    clf % clear figure

    sgtitle(txt,'Interpreter','latex')

    switch model

        case "standardrwdtractortrailer"
            % Standard tractor-trailer model
            xR = x_opt(1,i);
            yR = x_opt(2,i);
            theta = x_opt(3,i);
            psi = x_opt(4,i);
            psi2 = x_opt(5,i);
            phi = x_opt(6,i);

        case "pointp_rwdtractortrailer"
            % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
            % x = xR + L*cos(theta) + d*cos(theta+phi)
            % y = yR + L*sin(theta) + d*sin(theta+phi)

            xR = x_opt(1,i) - L*cos(x_opt(3,i)) - d*cos(x_opt(3,i)+x_opt(6,i));
            yR = x_opt(2,i) - L*sin(x_opt(3,i)) - d*sin(x_opt(3,i)+x_opt(6,i));
            theta = x_opt(3,i);
            psi = x_opt(4,i);
            psi2 = x_opt(5,i);
            phi = x_opt(6,i);

        otherwise
            error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
    end

    xF = xR+L*cos(theta);
    yF = yR+L*sin(theta);

    hitchJoint = [xR - L1*cos(theta),yR - L1*sin(theta)];
    xTrailer = hitchJoint(1) - L2*cos(theta+psi);
    yTrailer = hitchJoint(2) - L2*sin(theta+psi);

    hitchJoint2 = [xTrailer - L3*cos(theta+psi),yTrailer - L3*sin(theta+psi)];
    xTrailer2 = hitchJoint2(1) - L4*cos(theta+psi+psi2);
    yTrailer2 = hitchJoint2(2) - L4*sin(theta+psi+psi2);

    axis square, axis equal, grid on,...
        hold on

    plotArrow = 0;
    plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3)
    plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,hitchJoint2,plotArrow,0.3)
    plot_trailer(xTrailer2,yTrailer2,hitchJoint2,theta+psi,psi2,trailerWidth,L4,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,[],plotArrow,0.3)
    xlabel('$$x \, [m]$$','Interpreter','latex'), ylabel('$$y \, [m]$$','Interpreter','latex'),...

    %     % Desired padding value
    %     padding_val_x = 2.5;
    %     padding_val_y = padding_val_x;
    %
    %     if i < size(x_opt,2)
    %         % Center point
    %         center_x = x_pred(floor(N/2),1,i);
    %         center_y = x_pred(floor(N/2),2,i);
    %     else
    %         % Center point
    %         center_x = hitchJoint(1);
    %         center_y = hitchJoint(2);
    %     end
    %
    %     xlims = axis_padding(xlim,center_x,padding_val_x);
    %     ylims = axis_padding(ylim,center_y,padding_val_y);
    %
    %     % Set the new axis limits
    %     xlim(xlims);
    %     ylim(ylims);

    plot(x_ref(:),y_ref(:),'b','linewidth',0.5);

    r = wheelDiam/3;
    c1 = wheelRGB;
    c2 = "r";
    circles = plotFilledCircle(x_pred(1,1,i),x_pred(1,2,i),r,c1,c2);
    set(get(get(circles,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

    if plotGhost ~= 0
        if i < size(x_opt,2) % plot prediction
            %plot(x_pred(2:N-1,1,i),x_pred(2:N-1,2,i),'r:*')
            plot(x_pred(1:N,1,i),x_pred(1:N,2,i),'r:','linewidth',1.5)

            switch model

                case "standardrwdtractortrailer"
                    % Standard tractor-trailer model
                    xR = x_pred(N,1,i);
                    yR = x_pred(N,2,i);
                    theta = x_pred(N,3,i);
                    psi = x_pred(N,4,i);
                    phi = x_pred(N,5,i);

                case "pointp_rwdtractortrailer"
                    % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
                    % x = xR + L*cos(theta) + d*cos(theta+phi)
                    % y = yR + L*sin(theta) + d*sin(theta+phi)

                    xR = x_pred(N,1,i) - L*cos(x_pred(N,3,i)) - d*cos(x_pred(N,3,i)+x_pred(N,5,i));
                    yR = x_pred(N,2,i) - L*sin(x_pred(N,3,i)) - d*sin(x_pred(N,3,i)+x_pred(N,5,i));
                    theta = x_pred(N,3,i);
                    psi = x_pred(N,4,i);
                    phi = x_pred(N,5,i);

                    %                 theta = final_constraint(i,3);
                    %                 psi = final_constraint(i,4);
                    %                 phi = final_constraint(i,5);
                    %                 xR = x_pred(N,1,i) - L*cos(theta) - d*cos(theta+phi);
                    %                 yR = x_pred(N,2,i) - L*sin(theta) - d*sin(theta+phi);


                otherwise
                    error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
            end

            xF = xR+L*cos(theta);
            yF = yR+L*sin(theta);

            hitchJoint = [xR - L1*cos(theta),yR - L1*sin(theta)];
            xTrailer = hitchJoint(1) - L2*cos(theta+psi);
            yTrailer = hitchJoint(2) - L2*sin(theta+psi);

            plotArrow = 0;
            plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);
            plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);

            %                 switch model
            %
            %                     case "standardrwdtractortrailer"
            %                         % Standard tractor-trailer model
            %                         xR = x_pred(N,1,i);
            %                         yR = x_pred(N,2,i);
            %                         theta = x_pred(N,3,i);
            %                         psi = x_pred(N,4,i);
            %                         phi = x_pred(N,5,i);
            %
            %                     case "pointp_rwdtractortrailer"
            %                         % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
            %                         % x = xR + L*cos(theta) + d*cos(theta+phi)
            %                         % y = yR + L*sin(theta) + d*sin(theta+phi)
            %
            %                         %                 xR = x_pred(N,1,i) - L*cos(x_pred(N,3,i)) - d*cos(x_pred(N,3,i)+x_pred(N,5,i));
            %                         %                 yR = x_pred(N,2,i) - L*sin(x_pred(N,3,i)) - d*sin(x_pred(N,3,i)+x_pred(N,5,i));
            %                         %                 theta = x_pred(N,3,i);
            %                         %                 psi = x_pred(N,4,i);
            %                         %                 phi = x_pred(N,5,i);
            %
            %                         theta = final_constraint(i,3);
            %                         psi = final_constraint(i,4);
            %                         phi = final_constraint(i,5);
            %                         xR = final_constraint(i,1) - L*cos(theta) - d*cos(theta+phi);
            %                         yR = final_constraint(i,2) - L*sin(theta) - d*sin(theta+phi);
            %
            %
            %                     otherwise
            %                         error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
            %                 end
            %
            %                 xF = xR+L*cos(theta);
            %                 yF = yR+L*sin(theta);
            %
            %                 hitchJoint = [xR - L1*cos(theta),yR - L1*sin(theta)];
            %                 xTrailer = hitchJoint(1) - L2*cos(theta+psi);
            %                 yTrailer = hitchJoint(2) - L2*sin(theta+psi);
            %
            %                 plotArrow = 0;
            %                 plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);
            %                 plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);



            %plot([x_pred(1,1,i),x_pred(2,1,i)],[x_pred(1,2,i),x_pred(2,2,i)],'r:')
            circles = plotFilledCircle(x_pred(N,1,i),x_pred(N,2,i),r,c1,c2);
            set(get(get(circles,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            %plot([x_pred(N-1,1,i),x_pred(N,1,i)],[x_pred(N-1,2,i),x_pred(N,2,i)],'r:')

        end

    end

    hold off

    %pause(0.005)
    drawnow limitrate

    if ~ishandle(figh)
        % If the figure window is closed, stop the animation and exit the loop
        break;
    end

end

close(gcf) % close the figure
