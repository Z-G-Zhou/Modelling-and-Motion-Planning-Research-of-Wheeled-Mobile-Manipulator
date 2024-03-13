function [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_3priority(t,T)
% [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_double1(t,T) 生成末端轨迹、车轨迹和初始关节角
%   返回结果分别为规划的：末端轨迹，末端速度，车轨迹，车速度，初始关节角

    % 车轨迹可以跟踪

    % 周期及运动速率设置
    omega=2*pi/T;
        
    % 初始关节角
    q_0=[0 0 0.2 -pi/4 0.2 -pi/2 0.2 pi/4 0.2]';

    % 末端轨迹和末端速度
    R_ee=1.5;
    x = 0.731774020350448+R_ee*sin(omega*t);
    y = 0.187888634525836+R_ee*(1-cos(omega*t));
    z = 0.758053725051136;
    R = 0;
    P = 0;
    Y = 0;
    xyzrpy_d=[x;y;z;R;P;Y]; % 末端轨迹
  
    dx = R_ee*omega*cos(omega*t);
    dy = R_ee*omega*sin(omega*t);
    dz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    dxyzwxyz_d = [dx;dy;dz;wx;wy;wz]; % 末端速度
     
    % 车轨迹和车速度
    R_b=1.5;
    x1 = R_b*sin(omega*t);
    y1 = R_b*(1-cos(omega*t));
    th = 0;
    xyth_d=[x1;y1;th];
  
    dx1 = R_b*omega*cos(omega*t);
    dy1 = R_b*omega*sin(omega*t);
    dth = 0;
    dxyth_d = [dx1;dy1;dth];

end
