function [dy, ddy] = firstsecondderivatives(x,y)

% The function calculates the first & second derivative of a function that is given by a set
% of points. The first derivatives at the first and last points are calculated by
% the 3 point forward and 3 point backward finite difference scheme respectively.
% The first derivatives at all the other points are calculated by the 2 point
% central approach.
% The second derivatives at the first and last points are calculated by
% the 4 point forward and 4 point backward finite difference scheme respectively.
% The second derivatives at all the other points are calculated by the 3 point
% central approach.

n = length (x);
dy = zeros;
ddy = zeros;
% Input variables:
% x: vector with the x the data points.
% y: vector with the f(x) data points.
% Output variable:
% dy: Vector with first derivative at each point.
% ddy: Vector with second derivative at each point.
dy(1) = (-3*y(1) + 4*y(2) - y(3)) / (2*(x(2) - x(1))); % First derivative
ddy(1) = (2*y(1) - 5*y(2) + 4*y(3) - y(4)) / (x(2) - x(1))^2; % Second derivative

for i = 2:n-1
   
	dy(i) = (y(i+1) - y(i-1)) / (x(i+1) - x(i-1));
	ddy(i) = (y(i-1) - 2*y(i) + y(i+1)) / (x(i-1) - x(i))^2;
end

	dy(n) = (y(n-2) - 4*y(n-1) + 3*y(n)) / (2*(x(n) - x(n-1)));
	ddy(n) = (-y(n-3) + 4*y(n-2) - 5*y(n-1) + 2*y(n)) / (x(n) - x(n-1))^2;

end