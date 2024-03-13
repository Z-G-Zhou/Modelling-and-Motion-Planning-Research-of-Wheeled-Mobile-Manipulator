function [Px,Pv,Pa] = traj_plan_5x(Ax,Av,Aa,Bx,Bv,Ba,T,t)
% [Px,Pv,Pa] = traj_plan_5x(Ax,Av,Aa,Bx,Bv,Ba,T,t)   五次多项式轨迹规划
%   参数：起点A末点B的位置x，速度v，加速度a，总时间T，时刻t；
%   结果：t时刻时的Px,Pv,Pa
%   注：x，v，a可为标量向量如v=[vx,vy,vz]
%

    % 改为列向量
    Ax = reshape(Ax,[],1);
    Av = reshape(Av,[],1);
    Aa = reshape(Aa,[],1);
    Bx = reshape(Bx,[],1);
    Bv = reshape(Bv,[],1);
    Ba = reshape(Ba,[],1);

    %多项式系数
    a0=Ax;  
    a1=Av;
    a2=Aa/2;
    a3=(20.*Bx-20.*Ax-(8.*Bv+12.*Av).*T-(3.*Aa-Ba).*T.^2)./(2.*T.^3);
    a4=(30.*Ax-30.*Bx+(14.*Bv+16.*Av).*T+(3.*Aa-2.*Ba).*T.^2)./(2.*T.^4);
    a5=(12.*Bx-12.*Ax-(6.*Bv+6.*Av).*T-(Aa-Ba).*T.^2)./(2.*T.^5);

    % 结果
    Px=a0+a1.*t+a2.*t.^2+a3.*t.^3+a4.*t.^4+a5.*t.^5;
    Pv=a1+2.*a2.*t+3.*a3.*t.^2+4.*a4.*t.^3+5.*a5.*t.^4;
    Pa=2.*a2+6.*a3.*t+12.*a4.*t.^2+20.*a5.*t.^3;

end