function val = interpolation(xInterp,x,y)

for i=1:length(xInterp)-1
    index_end = find( x(:) > xInterp(i), 1, 'first');
    index_start  = find( x(:) <= xInterp(i), 1, 'last');
    
    val(i) = linearRegression(x(index_start),y(index_start),x(index_end),y(index_end),xInterp(i));
end

val(end+1) = y(end);

val = val';

end