function  []=ploy_target3(x_target,y_target,h,r)
%创建圆柱数据
[x,y,z] = cylinder(1,20);
x=x*r+x_target;
y=y*r+y_target;
z=z*h;

%画yuan柱体
hold on
surf(x,y,z,'EdgeColor','blue', ...
    'FaceColor','blue', 'FaceAlpha','0.05');
%画圆心
plot3(x_target,y_target,h,'*','Color','blue','MarkerSize',10);
end