function []=quadrotor(bias_x,bias_y,bias_z,pitch,roll)
%缩放系数
zoom=5;
%旋翼外框
% 定义圆柱体参数
radius_big = 0.1*zoom;    % 大圆柱半径
height_big = 0.02*zoom;   % 大圆柱高度
resolution = 10;   % 圆柱体的分辨率

% 创建大圆柱体
[X, Y, Z] = cylinder(radius_big, resolution);
Z = Z * height_big;

% 定义小圆柱体参数
radius_small = 0.8*radius_big;    % 小圆柱半径
height_small = 0.5*height_big;    % 小圆柱高度

% 创建小圆柱体
[X_small, Y_small, Z_small] = cylinder(radius_small, resolution);
Z_small = Z_small * height_small;

%%
% 绘制旋翼外框
bias=0.15*zoom* ...
    [1,1;
    -1,1;
    -1,-1;
    1,-1];

for i=1:4
    %外层
    x=X+bias(i,1);
    y=Y+bias(i,2);
    z=Z;
    hold on
    s_1(i)=surf(x, y, z,'FaceAlpha',0.7);
    
    %内层
    x_small=X_small+bias(i,1);
    y_small=Y_small+bias(i,2);
    z_small=Z_small;
    hold on
    s_2(i)=surf([x(2,:);x_small(2,:)],[y(2,:);y_small(2,:)],[z(2,:);z_small(2,:)],'FaceAlpha',0.7);
end

%%
%绘制机架
[x,y,z] = cylinder(1,50);
x=0.01*x*zoom;
y=0.01*y*zoom;
z=z*0.15*sqrt(2)*zoom;   

for i=1:4
    ss_2(i)=surf(x,y,z,'EdgeColor',"none", ...
    'FaceColor','black','FaceAlpha',0.35);
    origin=[0,0,0];
    direction = [1 0 0];
    rotate(ss_2(i),direction,90,origin)
    direction = [0,0,1];
    rotate(ss_2(i),direction,45+90*(i-1),origin)
end

%% 
%无人机姿态
%pitch，roll，机头朝向y
%^y
%|
%|---->x
for i=1:4
    %roll
    direction = [0,1,0];
    rotate(s_1(i),direction,roll,origin)
    rotate(s_2(i),direction,roll,origin)
    rotate(ss_2(i),direction,roll,origin)
    %pitch
    direction = [-1,0,0];
    rotate(s_1(i),direction,pitch,origin)
    rotate(s_2(i),direction,pitch,origin)
    rotate(ss_2(i),direction,pitch,origin)
    %bias_x
    s_1(i).XData=s_1(i).XData+bias_x;
    s_2(i).XData=s_2(i).XData+bias_x;
    ss_2(i).XData=ss_2(i).XData+bias_x;
    %bias_y
    s_1(i).YData=s_1(i).YData+bias_y;
    s_2(i).YData=s_2(i).YData+bias_y;
    ss_2(i).YData=ss_2(i).YData+bias_y;
    %bias_z
    s_1(i).ZData=s_1(i).ZData+bias_z;
    s_2(i).ZData=s_2(i).ZData+bias_z;
    ss_2(i).ZData=ss_2(i).ZData+bias_z;
end