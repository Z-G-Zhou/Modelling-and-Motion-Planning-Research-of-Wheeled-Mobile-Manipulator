function [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_ee_8(t,T)
% [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_ee_8(t,T) 生成末端轨迹和初始关节角
%   返回结果分别为规划的：末端轨迹，末端速度，车轨迹，车速度，初始关节角

    % 周期及画8速率设置
    omega=2*pi/T;
        
    % 初始关节角
    q_0=[0 0 0.2 -pi/4 0.2 -pi/2 0.2 pi/4 0.2]';

    % 末端轨迹和末端速度
    R_ee=1.5;
    x = 0.731774020350448+R_ee*sin(omega*t);
    y = 0.187888634525836-R_ee/2*sin(2*omega*t);
    z = 0.758053725051136;
    R = 0;
    P = 0;
    Y = 0;
    xyzrpy_d=[x;y;z;R;P;Y]; % 末端轨迹
  
    dx = R*omega*cos(omega*t);
    dy = -R*omega*cos(2*omega*t);
    dz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    dxyzwxyz_d = [dx;dy;dz;wx;wy;wz]; % 末端速度
     
    % 车轨迹和车速度
    xyth_d  = [0;0;0];   % 车轨迹
    dxyth_d = [0;0;0];   % 车速度

end
