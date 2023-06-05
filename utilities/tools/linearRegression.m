%% Linear regression for extrapolation function (or interpolation)

% Outside this interval, the formula is identical to linear extrapolation.
function y = linearRegression(x0,y0,x1,y1,x)
y = y0 + ((x - x0)/(x1 - x0))*(y1 - y0);
end