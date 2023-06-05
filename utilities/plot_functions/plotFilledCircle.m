function circles = plotFilledCircle(x,y,r,c1,c2)
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
circles = plot(x_circle, y_circle,'Color',c1);
fill(x_circle, y_circle, c2)
end