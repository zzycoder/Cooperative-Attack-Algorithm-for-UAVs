clc
clear all
close all

%%
figure
axis([0 50 0 50 0 10]);
axis equal
xlabel('x/(m)')
ylabel('y/(m)')
zlabel('z/(m)')
view(3)
grid on

%上色选择
rng(1)
all_colors = rand(6,3);
%%
%。。。。。。。数据记录。。。。。。
bag=zeros(6,300);
%%
%。。。。。。。数据记录。。。。。。
bag2=zeros(6,300);
%%
%。。。。。。。初始点-终止点。。。。。。。。
begin=[5,0];
over=[25,45];
%%
%。。。。。。。初始点-终止点。。。。。。。。
begin2=[40,0];
% over=[25,45];
%%
%。。。。。。。虚拟导引点。。。。。
r_gui=6;  %虚拟导引半径guide
theta_gui=-0.75*pi;   %虚拟导引点的夹角
v_target=0; %目标运动速度
over_gui=over+r_gui*[cos(theta_gui),sin(theta_gui)];  %给虚拟导引点赋值
%%
%。。。。。。。虚拟导引点。。。。。
% r_gui=5;  %虚拟导引半径guide
theta_gui2=-0.25*pi;   %虚拟导引点的夹角
% v_target=0; %目标运动速度
over_gui2=over+r_gui*[cos(theta_gui2),sin(theta_gui2)];  %给虚拟导引点赋值

%%
%。。。。。。。障碍物位置。。。。。。。。
obstacle=[15 35 10 24 40 20 21 35; ...
    15 15 30 30 25 15 30 30];
%%
%。。。。。。。是否可视化无人机。。。。。。。
vision_uav=1;
%%
%。。。。。。。是否可视化无人机。。。。。。。
vision_uav2=1;
%%
%。。。。。。。协同标志位。。。。。。。。
flag_co=0;  %cooperate
%%
%。。。。。。。协同标志位。。。。。。。。
flag_co2=0;  %cooperate
%%
accu=0;
accu2=0;
accu_time=0;
%%
%。。。。。。。初始化参数。。。。。。。。。。。
v=3;    %每次迭代的速度m/s
delta_t=0.1;    %时间间隔
t_end=200;  %最大仿真时间
iters=1;    %迭代次数初始化
%%
curr=[begin';0]; %当前位置
curr_previous=curr;    %上一刻位置
%%
curr2=[begin2';0]; %当前位置
curr_previous2=curr2;    %上一刻位置
%% 
testR=v*delta_t;   %测试圆的半径
Q_star=5;   %障碍物涉及的半径

num_point=36;       %周围的势能点个数
testPoint=zeros(num_point,2);   %周围8个点的坐标数组(x,y)
testOut=zeros(1,num_point); %周围8个点的势能？？
step_predict=10; %预测步长
%%
pos_predict=zeros(step_predict,3);  %预测域内的位置数组(x,y,z)，z信息固定
%%
pos_predict2=zeros(step_predict,3);  %预测域内的位置数组(x,y,z)，z信息固定

%%
%。。。。。。。预测点加上z轴信息。。。。。。。。
h=2;
pos_predict(:,3)=h; %无人机飞行定高为（m)
%%
%。。。。。。。预测点加上z轴信息。。。。。。。。
h2=4;
pos_predict2(:,3)=h2; %无人机飞行定高为（m)
%%
%。。。。。。。画无人机所需的参数。。。。。。。。。
roll_max=5;
pitch_max=5;
%%
U_k=zeros(3,1);
%%
U_k2=zeros(3,1);
%%
%。。。。。。。。画初始点。。。。。。。。
hold on;
plot3(begin(1),begin(2),0,'*b','MarkerSize',10);
%%
plot3(begin2(1),begin2(2),0,'*b','MarkerSize',10);
%%
%。。。。。。。画目标的圆形范围。。。。。。。。
plot_target(over(1),over(2),h,r_gui)
%%
%。。。。。。。画目标的圆形范围。。。。。。。。
plot_target(over(1),over(2),h2,r_gui)

%% 
%。。。。。。。。。MPC初始参数。。。。。。。。。。。
A=[zeros(3),eye(3);
    zeros(3),zeros(3)]*delta_t+ ...
    eye(6);   %状态矩阵A初始条件x_k,权重矩阵Q,R及终端误差矩阵F为输入

B=[0.5*eye(3)*delta_t^2;eye(3)*delta_t];    %输入矩阵B

N=step_predict;    %预测长度
%%
x_k=[begin(1);begin(2);0;
    0*ones(3,1)];   %当前状态，初始化
%%
x_k2=[begin2(1);begin2(2);0;
    0*ones(3,1)];   %当前状态，初始化
%%
Q=[eye(3),zeros(3);
    zeros(3),zeros(3)]; %权重矩阵Q

F=[eye(3),zeros(3);
    zeros(3),zeros(3)]; %权重矩阵，终端

R=[zeros(3)];   %权重矩阵，输入

u_max=3;    %最大输入加速度
ub=kron(ones(N,1),[u_max;u_max;u_max]); %输入上限
lb=-ub;

%%
%。。。。。。。主循环。。。。。。。。。。。。
while iters<=t_end/delta_t
    %计算当前点附近半径为testR的8个点的势能，然后让当前点的势能减去8个点的势能取差值最大的，
    %确定这个方向，就是下一步迭代的点
    %% 
    %。。。。。。。。。。对协同标志位更新。。。。。。。。。。
    if flag_co==1||norm(curr(1:2,1)'-over_gui)<2*testR
        %从0到1的切换是一次性的
        flag_co=1;  %可以进行近距离靠近
    end
    %% 
    %。。。。。。。。。。对协同标志位更新。。。。。。。。。。
    if flag_co2==1||norm(curr2(1:2,1)'-over_gui2)<2*testR
        %从0到1的切换是一次性的
        flag_co2=1;  %可以进行近距离靠近
    end
    %%
    if flag_co&&flag_co2
        accu_time=accu_time+1;
    end
    
    %%
    %。。。。。。。。。删除画图。。。。。。。。。。
    %%
    %删除当前点
    if vision_uav
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    %%
    %删除当前点
    if vision_uav2
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    %%
    %删除预测点序列
    delete(findobj('color',"green"));
    %删除障碍物
     delete(findobj('FaceColor','red'));
    %删除虚拟导引点
    delete(findobj('color',all_colors(5,:)));
%     %删除目标的圆形范围
%     delete(findobj('Color','blue'));
    %%
    %删除预测点序列
    delete(findobj('color',"magenta"));
%     %删除障碍物
%      delete(findobj('FaceColor','red'));
    %删除虚拟导引点
    delete(findobj('color',all_colors(5,:)));
%     %删除目标的圆形范围
%     delete(findobj('Color','blue'));

    %%
    %。。。。。。。。。导引点运动。。。。。。。。。。
    over_gui(1,1)=over_gui(1,1)+v_target*delta_t;
    %%
    %。。。。。。。。。导引点运动。。。。。。。。。。
    over_gui2(1,1)=over_gui2(1,1)+v_target*delta_t;
    %%
    %。。。。。。。。画垂直投影。。。。。。。。。
    plot3([curr(1),curr(1)],[curr(2),curr(2)],[curr(3),0], ...
        ':','Color',"#4DBEEE",'linewidth',1)
    %。。。。。。。。画位置的连线。。。。。。。。。
    plot3([curr_previous(1),curr(1)],[curr_previous(2),curr(2)],[curr_previous(3),curr(3)], ...
        '-','Color',"#4DBEEE",'linewidth',2)
    curr_previous=curr; %记录
    %。。。。。。。。画当前位置。。。。。。。。。
    %。。。。。。。。。。画无人机姿态。。。。。。。。
    if vision_uav
        %机头朝向y
        %沿x方向的加速度体现为滚转
        %沿y的加速度体现为俯仰
        roll=U_k(1,1)/u_max*roll_max; 
        pitch=U_k(2,1)/u_max*pitch_max;   
        quadrotor(curr(1),curr(2),curr(3),pitch,roll)  %调用函数画无人机
    %     plot3(curr(1),curr(2),curr(3),'o','Color',all_colors(2,:),'MarkerSize',5)
    end
    %%
    %。。。。。。。。画垂直投影。。。。。。。。。
    plot3([curr2(1),curr2(1)],[curr2(2),curr2(2)],[curr2(3),0], ...
        ':','Color',"#D95319",'linewidth',1)
    %。。。。。。。。画位置的连线。。。。。。。。。
    plot3([curr_previous2(1),curr2(1)],[curr_previous2(2),curr2(2)],[curr_previous2(3),curr2(3)], ...
        '-','Color',"#D95319",'linewidth',2)
    curr_previous2=curr2; %记录
    %。。。。。。。。画当前位置。。。。。。。。。
    %。。。。。。。。。。画无人机姿态。。。。。。。。
    if vision_uav2
        %机头朝向y
        %沿x方向的加速度体现为滚转
        %沿y的加速度体现为俯仰
        roll=U_k2(1,1)/u_max*roll_max; 
        pitch=U_k2(2,1)/u_max*pitch_max;   
        quadrotor(curr2(1),curr2(2),curr2(3),pitch,roll)  %调用函数画无人机
    %     plot3(curr(1),curr(2),curr(3),'o','Color',all_colors(2,:),'MarkerSize',5)
    end
    

%% 
    %。。。。。。。。。势场法求解预测点。。。。。。。。。。
    if accu_time<=2
        curr_temp=curr(1:2,1)';
        for i=1:step_predict
            %先求该位置上的八个点的坐标
            for j=1:num_point
                testPoint(j,:)=[testR*cos((j-1)*2*pi/num_point)+curr_temp(1), ...
                    testR*sin((j-1)*2*pi/num_point)+curr_temp(2)];
                testOut(:,j)=comput_P(testPoint(j,:)',over_gui',obstacle, Q_star);
                %找出来最小的就可以了
            end
            [~,num]=min(testOut);

            %迭代的位置
            curr_temp=testPoint(num,:);  
            %记录
            pos_predict(i,1:2)=curr_temp;    %对x，y赋值
        end
        
    %。。。。。。。。。直接指定协同角度。。。。。。。。。。
    else
        testR=v*delta_t*2/3;
        x_start=over_gui(1,1)-testR*cos(theta_gui)*accu;
        y_start=over_gui(1,2)-testR*sin(theta_gui)*accu;
        accu=accu+1;
        i=[1:step_predict]';   %创造好递增的数列
        %填入预测点列的坐标
        pos_predict(:,1)=x_start-testR*cos(theta_gui)*i;
        pos_predict(:,2)=y_start-testR*sin(theta_gui)*i;
    end
%% 

    %。。。。。。。。。势场法求解预测点。。。。。。。。。。
    if accu_time<=2
        curr_temp=curr2(1:2,1)';
        for i=1:step_predict
            %先求该位置上的八个点的坐标
            for j=1:num_point
                testPoint(j,:)=[testR*cos((j-1)*2*pi/num_point)+curr_temp(1), ...
                    testR*sin((j-1)*2*pi/num_point)+curr_temp(2)];
                testOut(:,j)=comput_P(testPoint(j,:)',over_gui2',obstacle, Q_star);
                %找出来最小的就可以了
            end
            [~,num]=min(testOut);

            %迭代的位置
            curr_temp=testPoint(num,:);  
            %记录
            pos_predict2(i,1:2)=curr_temp;    %对x，y赋值
        end
        
    %。。。。。。。。。直接指定协同角度。。。。。。。。。。
    else
        x_start=over_gui2(1,1)-testR*cos(theta_gui2)*accu2;
        y_start=over_gui2(1,2)-testR*sin(theta_gui2)*accu2;
        accu2=accu2+1;
        i=[1:step_predict]';   %创造好递增的数列
        %填入预测点列的坐标
        pos_predict2(:,1)=x_start-testR*cos(theta_gui2)*i;
        pos_predict2(:,2)=y_start-testR*sin(theta_gui2)*i;
    end
    
%%
    %。。。。。。。。。。画图。。。。。。。。。。
    %画预测集合
    plot3(pos_predict(:,1),pos_predict(:,2),pos_predict(:,3), ...
        'Color',"green",'linewidth',2)
    plot3([pos_predict(1,1),curr(1)],[pos_predict(1,2),curr(2)],[pos_predict(1,3),curr(3)], ...
        'Color',"green",'linewidth',2)
    %画障碍物
    for j=1:size(obstacle,2)
        plot_obstacle(obstacle(1,j),obstacle(2,j),Q_star/2,h2*1.5);
    end
    %画虚拟导引点
    plot3(over_gui(1),over_gui(2),h,'*','Color',all_colors(5,:),'MarkerSize',10);
%     %画目标的圆形范围
%     plot_target(over_gui(1),over_gui(2),h,r_gui,theta_gui);
%%
    %。。。。。。。。。。画图。。。。。。。。。。
    %画预测集合
    plot3(pos_predict2(:,1),pos_predict2(:,2),pos_predict2(:,3), ...
        'Color',"magenta",'linewidth',2)
    plot3([pos_predict2(1,1),curr2(1)],[pos_predict2(1,2),curr2(2)],[pos_predict2(1,3),curr2(3)], ...
        'Color',"magenta",'linewidth',2)
%     %画障碍物
%     for j=1:size(obstacle,2)
%         plot_obstacle(obstacle(1,j),obstacle(2,j),Q_star/2,h*2);
%     end
    %画虚拟导引点
    plot3(over_gui2(1),over_gui2(2),h2,'*','Color',all_colors(5,:),'MarkerSize',10);
%     %画目标的圆形范围
%     plot_target(over_gui2(1),over_gui2(2),h2,r_gui,theta_gui2);
    
    %% 
    %。。。。。。。MPC求解下一步。。。。。。。。。
    x_k_bias=zeros((N+1)*6,1);    %参考路径初始化
    x_k_bias(1:3,1)=[curr(1);curr(2);h];
    for k=1:N
        x_k_bias(6*k+1:6*k+3,1)=pos_predict(k,:)';
    end
    
    [M,C,U_k] = MPC(A,B,N,x_k,x_k_bias,Q,R,F,lb,ub);    %求解
    
    x=M*x_k+C*U_k;
    x_k=x(7:12,1);  %更新当前状态
    bag(:,iters)=x_k;    %记录数据
    curr=x_k(1:3,1);   %更新curr,3维
    %% 
    %。。。。。。。MPC求解下一步。。。。。。。。。
    x_k_bias=zeros((N+1)*6,1);    %参考路径初始化
    x_k_bias(1:3,1)=[curr2(1);curr2(2);h];
    for k=1:N
        x_k_bias(6*k+1:6*k+3,1)=pos_predict2(k,:)';
    end
    
    [M,C,U_k2] = MPC(A,B,N,x_k2,x_k_bias,Q,R,F,lb,ub);    %求解
    
    x=M*x_k2+C*U_k2;
    x_k2=x(7:12,1);  %更新当前状态
    bag2(:,iters)=x_k2;    %记录数据
    curr2=x_k2(1:3,1);   %更新curr,3维
    
    %%
    %。。。。。。。。暂停。。。。。。。。。。。。
    pause(0.01);
    iters=iters+1
    
end

