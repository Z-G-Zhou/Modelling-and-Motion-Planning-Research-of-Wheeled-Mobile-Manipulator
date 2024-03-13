function [Ti,T] = build_T(DH_table)
% [Ti,T] = build_T(DH_table) 由（改进）DH参数表生成相邻关节的Ti和基到末连杆系的T
%   （改进）DH参数表：n行四列，行标为连杆序号，列分别为扭角、长度、偏距、关节角
%    Ti：Ti(i)为从系i-1到系i的位姿矩阵^{i-1}_i T,  T(1)则是基系至1系
%    T ：基系至n系的总位姿矩阵
%    要求：1系相对基系，只有沿着y轴的d1的变化

[m,~]=size(DH_table);
Ti=zeros(4,4,m);     % 预分配空间
TT=eye(4);% 累乘变量

for i=1:m   % 2系到n系的T
    alphai_1=DH_table(i,1);  % i-1 连杆扭角
    ai_1=DH_table(i,2);      % i-1 连杆长度
    di=DH_table(i,3);           % i 关节偏距
    qi=DH_table(i,4);           % i 关节角
         
  Ti(1:4,1:4,i)=[  % 第i-1连杆系到i连杆系的位姿矩阵
    cos(qi)               ,-sin(qi)                 ,0                 ,ai_1              ;
    sin(qi)*cos(alphai_1) ,cos(qi)*cos(alphai_1)    ,-sin(alphai_1)    ,-sin(alphai_1)*di ;
    sin(qi)*sin(alphai_1) ,cos(qi)*sin(alphai_1)    ,cos(alphai_1)     ,cos(alphai_1)*di  ;
    0                     ,0                        ,0                 ,1                 ];

    TT=TT*Ti(1:4,1:4,i);
end
 T=TT;
%Ti;
    
end

