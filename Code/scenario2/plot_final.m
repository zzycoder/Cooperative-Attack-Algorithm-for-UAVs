%。。。。。。。。。画与目标的距离变化。。。。。。。。。。。
figure
axis([0 delta_t*iters 0 inf]);
grid on
line=1.5;

hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag(1:2,1:iters-1)-over'), ...
    'Color','#2084C5','linewidth',line)

hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag2(1:2,1:iters-1)-over'), ...
    'Color','#DE6836','linewidth',line)
hold on
yline(r_gui,'--','Threshold=6m');   %画辅助线
legend('uav1','uav2'); %画图例
xlabel('t/(s)')
ylabel('d/(m)')
%%
%。。。。。。。。画与障碍物的距离。。。。。。。。
figure
grid on
axis([0,delta_t*iters,0,inf])
line=1.5;

hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs(:,1:iters-1)-bag(1:2,1:iters-1)), ...
    'Color','#2084C5','linewidth',line)
hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs2(:,1:iters-1)-bag(1:2,1:iters-1)), ...
    'Color','#DE6836','linewidth',line)
hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs3(:,1:iters-1)-bag(1:2,1:iters-1)), ...
    'Color','#EDB120','linewidth',line)
hold on
yline(5,'--','Threshold');   %画辅助线
legend('d_{1}','d_{2}','d_{2}'); %画图例
xlabel('t/(s)')
ylabel('d/(m)')
%%
%。。。。。。。。画与障碍物的距离。。。。。。。。
figure
grid on
axis([0,delta_t*iters,0,inf])

hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs(:,1:iters-1)-bag2(1:2,1:iters-1)), ...
    'Color','#2084C5','linewidth',line)
hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs2(:,1:iters-1)-bag2(1:2,1:iters-1)), ...
    'Color','#DE6836','linewidth',line)
hold on
plot(delta_t*[1:iters-1], ...
    vecnorm(bag_obs3(:,1:iters-1)-bag2(1:2,1:iters-1)), ...
    'Color','#EDB120','linewidth',line)
yline(5,'--','Threshold');   %画辅助线
legend('d_{1}','d_{2}','d_{2}'); %画图例
xlabel('t/(s)')
ylabel('d/(m)')