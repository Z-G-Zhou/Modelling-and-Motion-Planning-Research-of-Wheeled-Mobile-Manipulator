XYZRPY_d=out.XYZRPY_d.data;         % 规划末端位置
dXYZWXYZ_d=out.dXYZWXYZ_d.data;     % 规划末端速度
XYTH_d=out.XYTH_d.data;             % 规划车位置
dXYTH_d=out.dXYTH_d.data;           % 规划车速度

Q_c=out.Q_c.data;           % 关节角度控制信号
dQ_c=out.dQ_c.data;         % 关节速度控制信号

Q=out.Q.data;             % 关节角度
dQ=out.dQ.data;           % 关节角速度

XYZRPY_W0=out.XYZRPY_W0.data;   % 车的位置（世界系）
XYZRPY_0E=out.XYZRPY_0E.data;   % 臂末端相对车的位置（车系）
XYZRPY_WE=out.XYZRPY_WE.data;   % 臂末端的位置（世界系）

dXYZWXYZ_W0=out.dXYZWXYZ_W0.data;       % 车的姿态矩阵（世界系）
dXYZWXYZ_0E=out.dXYZWXYZ_0E.data;        % 臂末端相对车的姿态矩阵（车系）
dXYZWXYZ_WE=out.dXYZWXYZ_WE.data;       % 臂末端的姿态矩阵（世界系）

tout=out.tout;            % 时间序列

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
