function []=plot_obstacle2(bias_x,bias_y,r,h)
%创建圆柱数据
[x,y,z] = cylinder(1,20);
x=x*r+bias_x;
y=y*r+bias_y;
z=z*h;

%画图
hold on
surf(x,y,z,'EdgeColor','red', ...
    'FaceColor','red', 'FaceAlpha','0.1');
% surf(x,y,z,'EdgeColor',"none", ...
%     'FaceColor','red', 'FaceAlpha','0.3');
% color = [1 0 0];
% colormap(color)