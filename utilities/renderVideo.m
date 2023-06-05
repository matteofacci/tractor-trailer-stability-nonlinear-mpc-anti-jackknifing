%% Draw/Render the Scenario

plotGhost = 0;
plotOptimizedTrajectory = 1;

if finalDuration

    if finalDuration >= sim_time
        finalDuration = sim_time;
    end
    % Define the number of frames in the animation
    numFrames = length(t);

    % Calculate the frame rate based on the final duration
    frameRate = numFrames / finalDuration;
else
    frameRate = 1/T;
end

%figh = figureFullScreen(100);
figh = figure(100);
set(gcf, 'Color', 'w');
%set(gcf,'Units','pixels','Position',[0 0 1280 720]);
screenSize = get(0, 'ScreenSize');
figWidth = 1280;
figHeight = 720;
% figWidth = 720;
% figHeight = 480;
figPos = [screenSize(3)/2 - figWidth/2, screenSize(4)/2 - figHeight/2, figWidth, figHeight];
set(gcf,'Units','pixels','Position', figPos);
txt = "Trajectory tracking   $$\rightarrow$$   Task duration : " + string(sim_time) + " [s]";

% switch trajectory
%
%     case "linear"
%         th = 0:0.1:15;
%         x_ref = 0.5*th; y_ref = 3*ones(length(th),1)';
%
%     case "circular"
%         rad = 1.25;
%         th = 0:pi/50:2*pi;
%         x_ref = rad * cos(th);
%         y_ref = 5 + rad * sin(th);
%
%     case "lemniscate"
%         th = 0:pi/50:2*pi;
%         a = 5;
%         b = 5; %1.25
%         x_ref = a*sin(th);
%         y_ref = b*sin(th).*cos(th);
%
%     otherwise
%         error("Invalid trajectory type. Valid options are 'Linear', 'Circular', 'Lemniscate'")
% end


th = 0:T:sim_time;
starting_time = t(1);

if RRT
    x_ref = q_ref(1:min(length(t), length(x_opt(3,:))),1)';
    y_ref = q_ref(1:min(length(t), length(x_opt(3,:))),2)';
else

    for k = 1:length(t)
        [~,x_temp,y_temp,~,~,~,~,~] = referenceTrajectory(starting_time,k,T,motion,trajectory);
        x_ref(k) = x_temp;
        y_ref(k) = y_temp;
    end
end


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
            phi = x_opt(5,i);

        case "pointp_rwdtractortrailer"
            % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
            % x = xR + L*cos(theta) + d*cos(theta+phi)
            % y = yR + L*sin(theta) + d*sin(theta+phi)

            xR = x_opt(1,i) - L*cos(x_opt(3,i)) - d*cos(x_opt(3,i)+x_opt(5,i));
            yR = x_opt(2,i) - L*sin(x_opt(3,i)) - d*sin(x_opt(3,i)+x_opt(5,i));
            theta = x_opt(3,i);
            psi = x_opt(4,i);
            phi = x_opt(5,i);

        otherwise
            error("Invalid trajectory type. Valid options are 'standardRWDTractorTrailer', 'pointP_RWDTractorTrailer'")
    end

    xF = xR+L*cos(theta);
    yF = yR+L*sin(theta);

    hitchJoint = [xR - L1*cos(theta),yR - L1*sin(theta)];
    xTrailer = hitchJoint(1) - L2*cos(theta+psi);
    yTrailer = hitchJoint(2) - L2*sin(theta+psi);

    if plotOptimizedTrajectory

        subplot(1,2,1)

        hold on
        axis square, axis equal, grid on,...
            xlabel('$$x \, [m]$$','Interpreter','latex'), ylabel('$$y \, [m]$$','Interpreter','latex'),...
            title('Optimized trajectory','Interpreter','latex')

        plotArrow = 1;
        plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
        plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
        %legend boxoff


        % Desired padding value
        padding_val_x = 1;
        padding_val_y = padding_val_x;

        % Center point
        center_x = hitchJoint(1);
        center_y = hitchJoint(2);

        xlims = axis_padding(xlim,center_x,padding_val_x);
        ylims = axis_padding(ylim,center_y,padding_val_y);

        % Set the new axis limits
        xlim(xlims);
        ylim(ylims);

        plot(x_ref(:),y_ref(:),'b--','linewidth',0.5); hold on % plot reference trajectory
        legend([tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer'),"Reference trajectory"],'Interpreter','latex','Location','northwest','AutoUpdate','off')
        %legend([tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer'),"Reference trajectory"],'Interpreter','latex','Location','best')
        %     lgd = legend('show');
        %     lgd.NumColumns = 2;

        if RRT

            rad = (0.045*2)/3;
            c1 = "k";
            c2 = "r";
            circles = plotFilledCircle(x_opt(1,1),x_opt(2,1),rad,c1,c2);
            % label = "Start";
            % labelpoints(x_opt(1,1),x_opt(2,1),label,'interpreter','latex','buffer',0.5);
            circles = plotFilledCircle(x_opt(1,end),x_opt(2,end),rad,c1,c2);
            % label = "Goal";
            % labelpoints(x_opt(1,end),x_opt(2,end),label,'interpreter','latex','buffer',0.5);

            for j = 1:N_obs
                obs_plot = drawPolygon(obs_reduced(j).coord,'-k', 'linewidth', 2);
                obs_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
                safe_obs_plot = drawPolygon(obs(j).coord,'--k', 'linewidth', 1);
                safe_obs_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
            end
        end


        % hold off

        %plot(12,3,'ro','linewidth',1.5)

        subplot(1,2,2)

        title('MPC behavior','Interpreter','latex')

    end

    axis square, axis equal, grid on,...
        hold on

    plotArrow = 0;
    plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
    plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3);
    xlabel('$$x \, [m]$$','Interpreter','latex'), ylabel('$$y \, [m]$$','Interpreter','latex'),...


    % Desired padding value
    padding_val_x = 2.5;
    padding_val_y = padding_val_x;

    % if i < size(x_opt,2)
    %     % Center point
    %     center_x = x_pred(floor(N/2),1,i);
    %     center_y = x_pred(floor(N/2),2,i);
    % else
    % Center point
    center_x = hitchJoint(1);
    center_y = hitchJoint(2);
    % end

    xlims = axis_padding(xlim,center_x,padding_val_x);
    ylims = axis_padding(ylim,center_y,padding_val_y);

    % Set the new axis limits
    xlim(xlims);
    ylim(ylims);

    plot(x_ref(:),y_ref(:),'b','linewidth',0.5);

    hold on

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

            %         switch model
            %
            %             case "standardrwdtractortrailer"
            %                 % Standard tractor-trailer model
            %                 xR = x_pred(N,1,i);
            %                 yR = x_pred(N,2,i);
            %                 theta = x_pred(N,3,i);
            %                 psi = x_pred(N,4,i);
            %                 phi = x_pred(N,5,i);
            %
            %             case "pointp_rwdtractortrailer"
            %                 % Model with point P as representative point of the vehicle, instead of the center of the tractor rear wheel.
            %                 % x = xR + L*cos(theta) + d*cos(theta+phi)
            %                 % y = yR + L*sin(theta) + d*sin(theta+phi)
            %
            %                 %                 xR = x_pred(N,1,i) - L*cos(x_pred(N,3,i)) - d*cos(x_pred(N,3,i)+x_pred(N,5,i));
            %                 %                 yR = x_pred(N,2,i) - L*sin(x_pred(N,3,i)) - d*sin(x_pred(N,3,i)+x_pred(N,5,i));
            %                 %                 theta = x_pred(N,3,i);
            %                 %                 psi = x_pred(N,4,i);
            %                 %                 phi = x_pred(N,5,i);
            %
            %                 theta = final_constraint(i,3);
            %                 psi = final_constraint(i,4);
            %                 phi = final_constraint(i,5);
            %                 xR = final_constraint(i,1) - L*cos(theta) - d*cos(theta+phi);
            %                 yR = final_constraint(i,2) - L*sin(theta) - d*sin(theta+phi);
            %
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
            %         plotArrow = 0;
            %         plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);
            %         plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.10);


            r = wheelDiam/3;
            c1 = wheelRGB;
            c2 = "r";
            circles = plotFilledCircle(x_pred(1,1,i),x_pred(1,2,i),r,c1,c2);
            set(get(get(circles,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            %plot([x_pred(1,1,i),x_pred(2,1,i)],[x_pred(1,2,i),x_pred(2,2,i)],'r:')
            circles = plotFilledCircle(x_pred(N,1,i),x_pred(N,2,i),r,c1,c2);
            set(get(get(circles,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            %plot([x_pred(N-1,1,i),x_pred(N,1,i)],[x_pred(N-1,2,i),x_pred(N,2,i)],'r:')

        end

        legend([tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer'),"Reference trajectory","Predicted trajectory",...
            tractorTrailerLegendLabels(plotArrow,'Predicted tractor','Predicted trailer')],...
            'Interpreter','latex','Location','northwest','Orientation','horizontal')
        %axis padded
        % split the legend into two columns and center it horizontally
        lgd = legend('show');
        lgd.NumColumns = 2;

    else

        r = wheelDiam/3;
        c1 = wheelRGB;
        c2 = "r";
        circles = plotFilledCircle(x_opt(1,i),x_opt(2,i),r,c1,c2);

        hold on
        set(get(get(circles,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

        legend([tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer'),"Reference trajectory"],...
            'Interpreter','latex','Location','northwest','Orientation','horizontal','AutoUpdate','off')

        %axis padded
        % split the legend into two columns and center it horizontally
        lgd = legend('show');
        lgd.NumColumns = 1;

    end

    if RRT

        rad = (0.045*2)/3;
        c1 = "k";
        c2 = "r";
        circles = plotFilledCircle(x_opt(1,1),x_opt(2,1),rad,c1,c2);
        %label = "Start";
        %labelpoints(x_opt(1,1),x_opt(2,1),label,'interpreter','latex','buffer',0.5);
        circles = plotFilledCircle(x_opt(1,end),x_opt(2,end),rad,c1,c2);
        %label = "Goal";
        %labelpoints(x_opt(1,end),x_opt(2,end),label,'interpreter','latex','buffer',0.5);

        for j = 1:N_obs
            obs_plot = drawPolygon(obs_reduced(j).coord,'-k', 'linewidth', 2);
            obs_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
            safe_obs_plot = drawPolygon(obs(j).coord,'--k', 'linewidth', 1);
            safe_obs_plot.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    end


    hold off

    movieVector(i) = getframe(figh);

end

close(gcf) % close the figure


%% Save the movie

videoName = sprintf('tractor_trailer_%s_%s', trajectory,datestr(now,'dd-mm-yyyy HH-MM'));

myWriter = VideoWriter(videoName, 'MPEG-4'); % generate the file curve.mp4

% If you want only one frame to appear per second, then you just need to
% set this rate in myWriter

myWriter.FrameRate = floor(frameRate);

% Open the VideoWriter object, write the movie, and close the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);

movefile('*mp4','videos');

