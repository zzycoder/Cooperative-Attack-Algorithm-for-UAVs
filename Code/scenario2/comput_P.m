function [ output ] = comput_P( curr,over,obstacle, Q_star)
k_att=1;
repu=0;
k_rep=1000;
Q_star=Q_star;
%计算当前点距离终点的引力
attr=1/2*k_att*(norm(curr-over))^2;

%计算障碍点与当前点的斥力
%设定障碍的斥力作用半径为2
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^(0.5);
    else
        repu=repu+0;
    end
end

output=attr+repu;


