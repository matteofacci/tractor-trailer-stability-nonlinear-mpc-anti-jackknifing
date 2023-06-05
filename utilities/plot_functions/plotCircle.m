function [x,y] = plotCircle(x,y,r,color,lineWidth)
theta = 0 : 0.01 : 2*pi;
x = r * cos(theta) + x;
y = r * sin(theta) + y;
plot(x, y,'Color',color,'LineWidth',lineWidth);
end