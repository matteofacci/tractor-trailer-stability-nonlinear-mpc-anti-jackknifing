function plot_tractor(xR,xF,yR,yF,hitchJoint,theta,phi,tractorWidth,tractorLenght,tractorBodyLenght,tractorRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,opacity)

%% Centered Bicycle
draw_rectangle([xR,yR],wheelWidth,wheelDiam,theta,wheelRGB,0.0);
draw_rectangle([xF,yF],wheelWidth,wheelDiam,theta+phi,wheelRGB,0.0);
%plot([xR xF], [yR yF],'LineWidth',3,'Color','r')
%scatter([xR xF], [yR yF],100,'filled','MarkerEdgeColor','r','MarkerFaceColor','r')

%% Body and wheels
xCenter = mean([xR,xF]);
yCenter = mean([yR,yF]);
[RL,RR,FL,FR] = draw_rectangle([xCenter,yCenter],tractorWidth-wheelWidth,tractorLenght,theta,tractorRGB,0.0); % wheels position
%draw_rectangle([xCenter,yCenter],tractorWidth+wheelWidth,tractorLenght+wheelDiam,theta,tractorRGB,0.3); % body
draw_rectangle([hitchJoint(1)+(tractorBodyLenght/2)*cos(theta),hitchJoint(2)+(tractorBodyLenght/2)*sin(theta)],tractorWidth,tractorBodyLenght,theta,tractorRGB,opacity); % body

% Rear Axle
draw_rectangle(RL,wheelWidth,wheelDiam,theta,wheelRGB,1);
draw_rectangle(RR,wheelWidth,wheelDiam,theta,wheelRGB,1);
p1 = plot([RL(1) RR(1)], [RL(2) RR(2)],'LineWidth',1.5,'Color',wheelRGB);
p1.Color(4) = 0.3;  % 70% transparent

% Front Axle
draw_rectangle(FL,wheelWidth,wheelDiam,theta+phi,wheelRGB,1);
draw_rectangle(FR,wheelWidth,wheelDiam,theta+phi,wheelRGB,1);
p2 = plot([FL(1) FR(1)], [FL(2) FR(2)],'LineWidth',1.5,'Color',wheelRGB);
p2.Color(4) = 0.3;  % 70% transparent

%% Hitch joint
p3 = plot([xR,hitchJoint(1)],[yR,hitchJoint(2)],'Color',wheelRGB, 'LineWidth',1.5);
p3.Color(4) = 0.3;  % 70% transparent
%scatter(hitchJoint(1),hitchJoint(2),30,'filled','MarkerEdgeColor',wheelRGB);
r = wheelWidth/2;
c1 = wheelRGB;
c2 = "b";
circles = plotFilledCircle(hitchJoint(1),hitchJoint(2),r,c1,c2);

if plotArrow
    %% Phi arrow
    arrowLength = tractorBodyLenght*0.5; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico
    angle = theta+phi;
    rho=arrowLength*ones(1,length(angle));
    [a,b] = pol2cart(angle,rho);
    quiver(xF,yF,a, b,0,'r','linewidth',1.5)

    %% Theta arrow
    arrowLength = tractorBodyLenght*0.5; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico
    angle = theta;
    rho=arrowLength*ones(1,length(angle));
    [a,b] = pol2cart(angle,rho);
    quiver(xR,yR,a, b,0,'y','linewidth',1.5)
end

end