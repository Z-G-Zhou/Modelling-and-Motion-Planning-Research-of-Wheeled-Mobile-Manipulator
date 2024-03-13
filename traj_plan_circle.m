function [P] = traj_plan_circle(A,B,C,T,t)
% [P] = traj_plan_circle(A,B,C,T,t) 圆弧轨迹规划
%   参数：起点A，中间点B，终点C 的三维坐标
%         总运动时间T，当前时间t
%   结果：当前空间位置P
%   注：P A B C 均为三维（列）向量

	% 改为列向量
    A = reshape(A,[],1);
    B = reshape(B,[],1);
    C = reshape(C,[],1);

    % 求圆心坐标和圆半径
    u1=B-A;
    w1=cross((C-A),u1);
    u=u1/norm(u1);
    w=w1/norm(w1);
    v=cross(w,u);
    bx=(B-A)*u';
    cx=(C-A)*u';
    cy=(C-A)*v';
    h=(cx^2+cy^2-bx*cx)/(2*cy);
    P0=A+(bx/2)*u+h*v;
    Ra=norm(P1-P0);
    
    % 求P0-RST坐标系到O-XYZ坐标系的变换关系T
    r1=A-P0;
    t1=cross(r1,B-P0);
    r=r1/norm(r1);
    t=t1/norm(t1);
    s=cross(t,r);
    R=[r',s',t'];
    T=[   R   , P0';
       [0,0,0],  1 ];
    iT=[  R'   ,-R'*P0';
        [0,0,0],   1  ];
    
    % 轨迹规划步骤
    OP=[A',B',C';  % 三个坐标点在O-XYZ坐标系的表示
        1 ,1 ,1  ]; 
    Pp=iT*OP; % 三个坐标点在P0-RST坐标系的表示 

    PpC=Pp(1:3,3)';% P0-RST坐标系下C三坐标点
    th13=atan2(PpC(2),PpC(1));
    if th13<0
        th13=th13+2*pi;
    end
    th=th13*t/T;  %得到t时刻对应的角度


    px=Ra*cos(th);
    py=Ra*sin(th);
    pz=0;
    PP=[px;py;pz;1]; %P系下的轨迹序列
    P=T*PP;P(4)=[]; % t时刻的P点
    
end