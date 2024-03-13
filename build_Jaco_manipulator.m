function Jaco = build_Jaco_manipulator(DH_table)
% function Jaco = build_Jaco_manipulator(DH_table) 由DH_table生成此时机械臂的雅可比
%   雅可比：关节速度与末端执行器广义速度的关系（相对于基座系）
%   矢量积法
%

[Ti,~] = build_T(DH_table); % 0-1,1-2, ... ,n-E 共DoF+1个Ti位姿矩阵
[m,~]=size(DH_table);
T_0i=zeros(4,4,m); % 初始化:0到i系的位姿矩阵，第一个为T-01，最后一个为T-0E
T_iE=zeros(4,4,m); % 初始化:i到E系的位姿矩阵，第一个为T-0E，最后一个为T-nE
T_0i(1:4,1:4,1)=Ti(1:4,1:4,1); % 初始化:0到i系的位姿矩阵，第一个为T-01，最后一个为T-0E
T_iE(1:4,1:4,m)=Ti(1:4,1:4,m); % 初始化:i到E系的位姿矩阵，第一个为T-0E，最后一个为T-nE
for i=2:m
    T_0i(1:4,1:4,i    )=T_0i(1:4,1:4,i-1)*Ti(1:4,1:4,i    );
    T_iE(1:4,1:4,m+1-i)=Ti(1:4,1:4,m+1-i)*T_iE(1:4,1:4,m+2-i);
end
p_iE=T_iE(1:3,4,2:end);
R_0i=T_0i(1:3,1:3,1:end-1);
z=[0 0 1]';
Jaco=zeros(6,m-1);
for i=1:m-1
Jaco(1:6,i)=[cross(R_0i(1:3,1:3,i)*z , R_0i(1:3,1:3,i)*p_iE(1:3,1,i));
                        R_0i(1:3,1:3,i)*z                            ];
end
end

