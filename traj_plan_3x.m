function [Px,Pv,Pa] = traj_plan_3x(Ax,Av,Bx,Bv,T,t)
% [Px,Pv] = traj_plan_3x(Ax,Av,Bx,Bv,T,t)   三次多项式轨迹规划
%   参数：起点A末点B的位置x，速度v，总时间T，时刻t；
%   结果：t时刻时的Px,Pv
%   注：x，v，a可为标量向量如v=[vx,vy,vz]
%

    % 改为列向量
    Ax = reshape(Ax,[],1);
    Av = reshape(Av,[],1);
    Bx = reshape(Bx,[],1);
    Bv = reshape(Bv,[],1);

    %多项式系数
    a0=Ax;  
    a1=Av;
    a2=(3.*Bx-3.*Ax-(Bv+2.*Av).*T)./T.^2;
    a3=(2.*Ax-2.*Bx+(Bv+Av).*T)./T.^3;

    % 结果
    Px=a0+a1.*t+a2.*t.^2+a3.*t.^3;
    Pv=a1+2.*a2.*t+3.*a3.*t.^2;
    Pa=2.*a2+6.*a3.*t;

end