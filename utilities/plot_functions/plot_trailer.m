function plot_trailer(xR,yR,hitchJoint,theta,psi,trailerWidth,trailerLenght,trailerBodyLenght,trailerRGB,wheelWidth,wheelDiam,wheelRGB,plotArrow,opacity)

%% Body and wheels
xCenter = mean([xR,hitchJoint(1)]);
yCenter = mean([yR,hitchJoint(2)]);
[RL,RR,~,~] = draw_rectangle([xCenter,yCenter],trailerWidth-wheelWidth,trailerLenght,theta+psi,trailerRGB,0.0); % wheels position
%draw_rectangle([xR,yR],trailerWidth+wheelWidth,trailerLenght+(wheelDiam/2),theta+psi,trailerRGB,0.3); % body
draw_rectangle([xR,yR],trailerWidth,trailerBodyLenght,theta+psi,trailerRGB,opacity); % body

% Rear Axle
draw_rectangle(RL,wheelWidth,wheelDiam,theta+psi,wheelRGB,1);
draw_rectangle(RR,wheelWidth,wheelDiam,theta+psi,wheelRGB,1);
p1 = plot([RL(1) RR(1)], [RL(2) RR(2)],'LineWidth',1.5,'Color',wheelRGB);
p1.Color(4) = 0.3;  % 70% transparent

% Middle Axle
p2 = plot([xR hitchJoint(1)], [yR hitchJoint(2)],'LineWidth',1.5,'Color',wheelRGB);
p2.Color(4) = 0.3;  % 70% transparent

if plotArrow
    %% Psi arrow
    arrowLength = trailerBodyLenght*0.5; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico
    angle = theta+psi;
    rho=arrowLength*ones(1,length(angle));
    [a,b] = pol2cart(angle,rho);
    quiver(hitchJoint(1),hitchJoint(2),a, b,0,'b','linewidth',1.5)
end

end