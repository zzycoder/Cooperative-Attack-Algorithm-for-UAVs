function [M,C,U_k] = MPC(A,B,N,x_k,x_k_bias,Q,R,F,lb,ub)
 
%%%%%%%%%%%建立一个以0为参考目标的MPC求解函数
%%%%%%%%%%%其中，状态矩阵A，输入矩阵B系统维度N，初始条件x_k,权重矩阵Q,R及终端误差矩阵F为输入
%%%%%%%%%%%输出中U_k为所求控制器，其余为简化过程中引入的中间变量
 
n=size(A,1); %A是n×n矩阵，求n
p=size(B,2); %B是n×p矩阵，求p
M=[eye(n);zeros(N*n,n)];%初始化M矩阵，M矩阵是(N+1)n × n的，
                        %它上面是n × n个“I”，这一步先把下半部分写成0
C=zeros((N+1)*n,N*p);%初始化C矩阵，这一步令它有(N+1)n × NP个0
%定义M和C
tmp=eye(n);%定义一个n × n的I矩阵
for i=1:N%循环，i从1到N
    rows = i*n+(1:n);%定义当前行数，从i×n开始，共n行
    C(rows,:)=[tmp*B,C(rows-n,1:end-p)];%将C矩阵填满
    tmp=A*tmp;%每一次将tmp左乘一次A
    M(rows,:)=tmp;%将M矩阵写满
end
%定义Q_bar
S_q=size(Q,1);%找到Q的维度
S_r=size(R,1);%找到R的维度
Q_bar=zeros((N+1)*S_q,(N+1)*S_q);%初始化Q_bar为全0矩阵
for i=0:N
    Q_bar(i*S_q+1:(i+1)*S_q,i*S_q+1:(i+1)*S_q)=Q;%将Q写到Q_bar的对角线上
end
Q_bar(N*S_q+1:(N+1)*S_q,N*S_q+1:(N+1)*S_q)=F;%将F放在最后一个位置
 
%定义R_bar
R_bar=zeros(N*S_r,N*S_r);%初始化R_bar为全0矩阵
for i=0:N-1
    R_bar(i*S_r+1:(i+1)*S_r,i*S_r+1:(i+1)*S_r)=R;
end
 
%求解
G=M'*Q_bar*M;%G
E=C'*Q_bar*M;%E
H=C'*Q_bar*C+R_bar;%H
%最优化
options = optimoptions('quadprog','Display','off');
f=(x_k'*E')'-(x_k_bias'*Q_bar*C)';%定义f矩阵
U_k=quadprog(H,f,[],[],[],[],lb,ub,[],options);%用二次规划求解最优化U_k
