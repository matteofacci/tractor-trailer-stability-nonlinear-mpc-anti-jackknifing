function x0 = initialAlignment(trajectory,motion,current_time,N,T)

for k = 1:N
    [t_predict,x_ref,y_ref,~,~,~,~,~] = referenceTrajectory(current_time,k,T,motion,trajectory);
    q(k,:) = [x_ref,y_ref];
    t_aux(k) = t_predict;
end

[dx, ~] = firstsecondderivatives(t_aux,q(:,1));
[dy, ~] = firstsecondderivatives(t_aux,q(:,2));

if motion == "backward"
    dx = -dx;
    dy = -dy;
end

tau = atan2(dy, dx);
indx = dx==0 & dy==0;
tau(indx) = tau(find(~indx,1,'last'));

x0 = [q(1,1);q(1,2);tau(1);0;0;0];

end