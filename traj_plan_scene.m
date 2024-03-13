function [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_scene(t,T)
% [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_scene（t,T） 生成末端轨迹、车轨迹和初始关节角
%   返回结果分别为规划的：末端轨迹，末端速度，车轨迹，车速度，初始关节角

% 初始关节角
q_0=[0 0 0.1 -0.733 0.1 -1.05 0.1 -0.59 0.1]';    

%% 末端轨迹点,车轨迹点
    p_e=[
        0.744358653349698 0.114296171343550 0.763038394397254;
        4   0   0.8;
        6   2   0.2;
        7.7 4   0.8;
        8   5   0.8;
        8.3 6   0.8;
        9.2 7   0.8;
        9.5 8.6 0.6;
        9   9   0.2;
        8.5 9.5 0.6;
        7   9   0.6;
        4   8   0.24;
        3   8   0.2;
        2   8   0.24;
        1   9.5 0.6;
        ];

    p_b=[
        0       0;
        0.2     0;
        0.5     0;
        4       0;
        6.3     1.7;
        7.9     4.5;
        8       5.2;
        8.1     5.7;
        8.3     6;
        9.3     7;
        9.6     8.6;
        9.4     9.4;
        8.5     9.6;
        7.5     9.2;
        4.5     8.1;
        3       8;
        2.5     8.1;
        1       9;
        ];

    n_pe=size(p_e,1); % 末端轨迹点的个数
    n_pb=size(p_b,1); % 小车轨迹点的个数
%% 末端和车两临近点的：位矢,距离，时间
    x_e=p_e(2:end,:)-p_e(1:end-1,:);% 末端位置矢量
    x_b=p_b(2:end,:)-p_b(1:end-1,:);% 小车位置矢量

    s_e=zeros(1,n_pe-1);
    for i=1:n_pe-1
        s_e(i)=sqrt(x_e(i,1)^2+x_e(i,2)^2+x_e(i,3)^2);% 末端轨迹点间的距离
    end
    S_e=sum(s_e);

    s_b=zeros(1,n_pb-1);
    for i=1:n_pb-1
        s_b(i)=sqrt(x_b(i,1)^2+x_b(i,2)^2);% 小车轨迹点间的距离
    end
    S_b=sum(s_b);

    T_e=s_e./S_e.*T; % 末端轨迹段的时间
    T_b=s_b./S_b.*T; % 小车轨迹段的时间

    T_b_error=zeros(1,n_pb-1); % 小车轨迹段的时间修正

    if sum(T_b_error)~=0
        error('T_b_error)~=0');
    end
    T_b=T_b+T_b_error;

%% 末端轨迹点和小车轨迹点的速度
    v_e_unit=[[0 0 0];x_e]+[x_e;[0 0 0]];
    v_e=zeros(n_pe,3);
    s_ee=[0,s_e,0];
    T_ee=[0,T_e,0];
    for i=1:n_pe
        v_e_unit(i,:)=v_e_unit(i,:)./sqrt(v_e_unit(i,1)^2+v_e_unit(i,2)^2+v_e_unit(i,3)^2);
        v_e(i,:)=v_e_unit(i,:).*(s_ee(i)+s_ee(i+1))./(T_ee(i)+T_ee(i+1));
    end


    v_b_unit=[[0 0];x_b]+[x_b;[0 0]];
    v_b=zeros(n_pb,2);
    s_bb=[0,s_b,0];
    T_bb=[0,T_b,0];
    for i=1:n_pb
        v_b_unit(i,:)=v_b_unit(i,:)./sqrt(v_b_unit(i,1)^2+v_b_unit(i,2)^2);
        v_b(i,:)=v_b_unit(i,:).*(s_bb(i)+s_bb(i+1))./(T_bb(i)+T_bb(i+1));
    end

%% 当前t时刻在第i段轨迹内
    t_this=t;
    for i_e=1:n_pe-1 % 末端在第i_e段
        if (t_this-sum(T_e(1:i_e)))<=0
            break;
        end
    end

    t_this=t;
    for i_b=1:n_pb-1 % 小车在第i_b段
        if (t_this-sum(T_b(1:i_b)))<=0
            break;
        end
    end

%% 输出轨迹
    % 末端
    if i_e==1 % 若是第一段
        te_this=t;         % 当前轨迹段的t
    else
        te_this=t-sum(T_e(1:(i_e-1)));
    end

    Te_this=T_e(i_e);% 当前轨迹段的总运动时间T
    Ax_this=p_e(i_e,:); % 当前轨迹段的起点位置
    Av_this=v_e(i_e,:); % 当前轨迹段的起点速度
    Bx_this=p_e(i_e+1,:); % 当前轨迹段的起点位置
    Bv_this=v_e(i_e+1,:); % 当前轨迹段的起点速度

    % 求解t时候的位置，速度，加速度
    [Pex,Pev,Pea] = traj_plan_3x(Ax_this,Av_this,Bx_this,Bv_this,Te_this,te_this);

    xyz=Pex;
    R = 0;
    P = 0;
    Y = 0;
    xyzrpy_d=[xyz;R;P;Y]; % 末端轨迹

    dxyz=Pev;
    wx = 0;
    wy = 0;
    wz = 0;
    dxyzwxyz_d = [dxyz;wx;wy;wz]; % 末端速度



    % 小车
    if i_b==1 % 若是第一段
        tb_this=t;         % 当前轨迹段的t
    else 
        tb_this=t-sum(T_b(1:(i_b-1)));
    end

    Tb_this=T_b(i_b);% 当前轨迹段的总运动时间T
    Ax_this=p_b(i_b,:); % 当前轨迹段的起点位置
    Av_this=v_b(i_b,:); % 当前轨迹段的起点速度
    Bx_this=p_b(i_b+1,:); % 当前轨迹段的起点位置
    Bv_this=v_b(i_b+1,:); % 当前轨迹段的起点速度

    % 求解t时候的位置，速度，加速度
    [Pbx,Pbv,Pba] = traj_plan_3x(Ax_this,Av_this,Bx_this,Bv_this,Tb_this,tb_this);

    xy=Pbx;
    th = 0;
    xyth_d=[xy;th];% 车轨迹

    dxy=Pbv;
    dth = 0;
    dxyth_d = [dxy;dth]; % 车速度

end
