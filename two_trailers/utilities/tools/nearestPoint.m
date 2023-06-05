function [nearest_point,index] = nearestPoint(x,y,target_x,target_y)

% Number of points
n = length(x);

% Calculate the Euclidean distance between the target coordinate and each point
distance = zeros(n, 1);
for i = 1:n
    distance(i) = sqrt((x(i) - target_x)^2 + (y(i) - target_y)^2);
end

% Find the index of the minimum distance
[~, index] = min(distance);

% Get the nearest point
nearest_point = [x(index), y(index)];

end