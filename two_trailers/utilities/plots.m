%% Plot the results

labelFontSize = 18;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultColorbarTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
% set(groot, 'defaultFigureRenderer','painters');
set(groot,'defaultAxesFontSize',labelFontSize)

starting_time = t(1);

for k = 1:length(t)
    [~,x_temp,y_temp,~,~,~,~,~] = referenceTrajectory(starting_time,k,T,motion,trajectory);
    x_ref(k) = x_temp;
    y_ref(k) = y_temp;
end

% Truncate vectors to the same length
max_length = min(length(t), length(x_opt(3,:)));
t = t(1:max_length);
xx_temp(:,:) = x_opt(:,1:max_length);
x_opt = xx_temp;

%% Plot errors

figh = figureFullScreen(1);
set(gcf, 'Color', 'w');
name = strcat(motion,'_',trajectory,'_',model,'_CH_',string(MPC_control_horizon),'_errors');
set(gcf, 'Name', name);

subplot(3,1,1);
hold on
plot(t,x_ref,'b-','linewidth',1.5)
plot(t,x_opt(1,:),'r--','linewidth',1.5)
%ylim([0,1])
if max(abs(x_ref))>0.5
    ylim('padded')
else
    ylim([-0.5,0.5])
end
xlim([t(1),t(end)])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$x [m]$$','Interpreter','latex')
title('$$x(t)$$','Interpreter','latex','FontSize', 25)
%title('$$e_x$$','Interpreter','latex')
legend(["Reference trajectory","Resulting trajectory"],'Interpreter','latex','Orientation','horizontal','Location','best')

% subplot(5,1,2);
% hold on
% plot(t,x_ref-x_opt(1,:),'r-','linewidth',1.5)
%
% if max(abs(x_ref-x_opt(1,:)))>0.5
%     ylim('padded')
% else
%     ylim([-0.5,0.5])
% end
%
% xlim([t(1),t(end)])
% grid on
% xlabel('time $$[s]$$','Interpreter','latex')
% ylabel('$$e_{x}\, [m]$$','Interpreter','latex')
% %title('$$e_{x}$$','Interpreter','latex')
% legend(["$$e_{x}$$"],'Interpreter','latex','Orientation','horizontal','Location','best')

subplot(3,1,2);
hold on
plot(t,y_ref,'b-','linewidth',1.5)
plot(t,x_opt(2,:),'r--','linewidth',1.5)
%ylim([0,1])
if max(abs(y_ref))>0.5
        ylim('padded')
    else
        ylim([-0.5,0.5])
    end
    xlim([t(1),t(end)])
    grid on
    xlabel('time $$[s]$$','Interpreter','latex')
    ylabel('$$y [m]$$','Interpreter','latex')
    title('$$y(t)$$','Interpreter','latex','FontSize', 25)
    %title('$$e_y$$','Interpreter','latex')
    legend(["Reference trajectory","Resulting trajectory"],'Interpreter','latex','Orientation','horizontal','Location','best')

% subplot(5,1,4);
% hold on
% plot(t,y_ref-x_opt(2,:),'r-','linewidth',1.5)
%
% if max(abs(y_ref-x_opt(2,:)))>0.5
%     ylim('padded')
% else
%     ylim([-0.5,0.5])
% end
%
% xlim([t(1),t(end)])
% grid on
% xlabel('time $$[s]$$','Interpreter','latex')
% ylabel('$$e_{y} \, [m]$$','Interpreter','latex')
% %title('$$e_{y}$$','Interpreter','latex')
% legend(["$$e_{y}$$"],'Interpreter','latex','Orientation','horizontal','Location','best')

subplot(3,1,3);
hold on
plot(t,position_error,'r-','linewidth',1.5)
xline(settling_time,'-.k',{'Settling time',"t = " + num2str(settling_time) + " [s]"},'LabelOrientation','horizontal','Interpreter','latex','linewidth',1.5)

if max(abs(position_error))>0.5
        ylim('padded')
    else
        ylim([-0.1,0.5])
    end

    xlim([t(1),t(end)])
    grid on
    xlabel('time $$[s]$$','Interpreter','latex')
    ylabel('$$e_{pos}\, [m]$$','Interpreter','latex')
    %title('$$e_{x}$$','Interpreter','latex')
    title('$$e_{pos}(t)$$','Interpreter','latex','FontSize', 25)
    legend(["$$e_{pos}$$"],'Interpreter','latex','Orientation','horizontal','Location','best')

%{
subplot(7,1,1);
hold on
plot(t,error(:,1),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_x$$','Interpreter','latex')
title('$$e_x$$','Interpreter','latex')

subplot(7,1,2);
hold on
plot(t,error(:,2),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_y$$','Interpreter','latex')
title('$$e_y$$','Interpreter','latex')

subplot(7,1,3);
hold on
plot(t,error(:,3),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_{\theta}$$','Interpreter','latex')
title('$$e_{\theta}$$','Interpreter','latex')

subplot(7,1,4);
hold on
plot(t,error(:,4),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_{\psi}$$','Interpreter','latex')
title('$$e_{\psi}$$','Interpreter','latex')

subplot(7,1,5);
hold on
plot(t,error(:,5),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_{\phi}$$','Interpreter','latex')
title('$$e_{\phi}$$','Interpreter','latex')

subplot(7,1,6);
hold on
plot(t,error(:,6),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_v$$','Interpreter','latex')
title('$$e_v$$','Interpreter','latex')

subplot(7,1,7);
hold on
plot(t,error(:,7),'r-','linewidth',1.5)
ylim([0,1])
grid on
xlabel('time $$[s]$$','Interpreter','latex')
ylabel('$$e_{\omega}$$','Interpreter','latex')
title('$$e_{\omega}$$','Interpreter','latex')

%}


%% Plot trajectory and optimal control

figh = figureFullScreen(2);
    set(gcf, 'Color', 'w');
    name = strcat(motion,'_',trajectory,'_',model,'_CH_',string(MPC_control_horizon),'_state_control');
    set(gcf, 'Name', name);

    subplot(2,1,1);
    plot(t,u_opt(:,1),'r-','linewidth',1.5);
    %stairs(t,u_opt(:,1),'r-','linewidth',1.5);
    hold on
    yline(v_max,'-.k',"Upper bound",'LabelOrientation','horizontal','LabelVerticalAlignment','middle','Interpreter','latex','linewidth',1)
    yline(v_min,'-.k',"Lower bound",'LabelOrientation','horizontal','LabelVerticalAlignment','middle','Interpreter','latex','linewidth',1)
     %stairs(t,v_aux,'b--','linewidth',1.5);


    % if max(abs(u_opt(:,1)))>= maximum_vel
        ylim('padded')
    % else
    %     ylim([minimum_vel,maximum_vel])
    % end

    xlim([t(1),t(end)])
    grid on
    xlabel('time $$[s]$$','Interpreter','latex')
    ylabel('$$v\, [m \times s^{-1}]$$','Interpreter','latex')
    title('$$v(t)$$','Interpreter','latex','FontSize', 25)

    subplot(2,1,2);
    plot(t,u_opt(:,2),'r-','linewidth',1.5);
    %stairs(t,u_opt(:,2),'r-','linewidth',1.5);
    hold on
    %stairs(t,omega_aux,'b--','linewidth',1.5);
    grid on

    yline(omega_max,'-.k',"Upper bound",'LabelOrientation','horizontal','LabelVerticalAlignment','middle','Interpreter','latex','linewidth',1)
    yline(omega_min,'-.k',"Lower bound",'LabelOrientation','horizontal','LabelVerticalAlignment','middle','Interpreter','latex','linewidth',1)
    % 
    % if max(abs(u_opt(:,2)))>= omega_max
        ylim('padded')
    % else
    %     ylim([omega_min,omega_max])
    % end

    xlim([t(1),t(end)])
    xlabel('time $$[s]$$','Interpreter','latex')
    ylabel('$$\omega \, [rad \times s^{-1}]$$','Interpreter','latex')
    title('$$\omega(t)$$','Interpreter','latex','FontSize', 25)

    % subplot(3,1,3);
    % plot(t,x_opt(3,:),'r-','linewidth',1.5)
    % grid on
    % ylim('padded')
    % xlim([t(1),t(end)])
    % xlabel('time $$[s]$$','Interpreter','latex')
    % ylabel('$$\phi \, [rad]$$','Interpreter','latex')
    % title('$$\phi(t)$$','Interpreter','latex')


%% Plot trajectory and optimal control

figh = figureFullScreen(3);
set(gcf, 'Color', 'w');
name = strcat(motion,'_',trajectory,'_',model,'_CH_',string(MPC_control_horizon),'_trajectory');
set(gcf, 'Name', name);

hold on
plot(x_ref(:),y_ref(:),'b','linewidth',1); hold on
plot(x_opt(1,:),x_opt(2,:),'r--','linewidth',1.5)
axis square, axis equal, grid on,... %xlim('padded'),ylim('padded'),...
    xlabel('$$x \, [m]$$','Interpreter','latex'), ylabel('$$y \, [m]$$','Interpreter','latex'),...
    % title('Optimized trajectory','Interpreter','latex')
axis padded

indices = round(linspace(1, length(x_opt), 8)); % select n equally spaced indices
k=1;

for i = 1:length(x_opt)

    if ismember(i, indices(1:end-1)) % Check if the current index is in the array

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

        plotArrow = 0;
        plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,L,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,0.3)
        plot_trailer(xTrailer,yTrailer,hitchJoint,theta,psi,trailerWidth,L2,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,hitchJoint2,plotArrow,0.3)
        plot_trailer(xTrailer2,yTrailer2,hitchJoint2,theta+psi,psi2,trailerWidth,L4,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,[],plotArrow,0.3)

        r = wheelDiam/3;
        c1 = wheelRGB;
        c2 = "r";
        circles = plotFilledCircle(x_pred(1,1,i),x_pred(1,2,i),r,c1,c2);
        %labels(k) = 'Point' + " " + num2str(k);
        %labels(k) = 'P' + " "  + num2str(k);
        labels(k) = "t = " + num2str(t(i)) + " [s]";
        labelpoints(xTrailer,yTrailer,labels(k),'interpreter','latex','buffer',1,'FontSize', labelFontSize);
        k = k+1;
    end
end
legend(["Reference trajectory","Resulting trajectory",tractorTrailerLegendLabels(plotArrow,'Tractor','Trailer')],'Interpreter','latex','Orientation','horizontal','Location','bestoutside')

% %% Plot errors
%
% figh = figureFullScreen(4);
% set(gcf, 'Color', 'w');
% name = strcat(motion,'_',trajectory,'_',model,'_CH_',string(MPC_control_horizon),'_errors');
% set(gcf, 'Name', name);
%
% for i = 1:length(final_constraint)
%     predicted_final_output(i,:) = x_pred(N,:,i);
% end
%
% subplot(3,1,1);
% hold on
% plot(t,(predicted_final_output(:,3)-final_constraint(:,3)),'b-','linewidth',1.5)
% %ylim([0,1])
% %ylim('padded')
% xlim([t(1),t(end)])
% grid on
% xlabel('time $$[s]$$','Interpreter','latex')
% %ylabel('$$x [m]$$','Interpreter','latex')
% title('$$e_{\theta}$$','Interpreter','latex')
%
% subplot(3,1,2);
% hold on
% plot(t,(predicted_final_output(:,4)-final_constraint(:,4)),'b-','linewidth',1.5)
% %ylim([0,1])
% %ylim('padded')
% xlim([t(1),t(end)])
% grid on
% xlabel('time $$[s]$$','Interpreter','latex')
% %ylabel('$$x [m]$$','Interpreter','latex')
% title('$$e_{\psi}$$','Interpreter','latex')
%
% subplot(3,1,3);
% hold on
% plot(t,(predicted_final_output(:,5)-final_constraint(:,5)),'b-','linewidth',1.5)
% %ylim([0,1])
% %ylim('padded')
% xlim([t(1),t(end)])
% grid on
% xlabel('time $$[s]$$','Interpreter','latex')
% %ylabel('$$x [m]$$','Interpreter','latex')
% title('$$e_{\phi}$$','Interpreter','latex')

%% Save pics

if saveFigs
    saveFigures(format);
end