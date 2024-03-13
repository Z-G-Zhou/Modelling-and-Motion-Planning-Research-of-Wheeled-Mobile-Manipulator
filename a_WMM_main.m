clc;clear;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 初始化
    tt=10;  dt=0.01;            % 时间-间隔
    
    TT=dt:dt:tt;

    xyth=[0.4834 0.1809 0.4]';  % 车的实际位移
    XYTH(3,round(tt/dt))=0;     % 车的实际位移

    dxyth=[0 0 0]';             % 车的实际速度
    dXYTH(3,round(tt/dt))=0;    % 车的实际速度

    xyzrpy=[0 0 0 0 0 0]';      % 臂的实际位移
    XYZRPY(6,round(tt/dt))=0;   % 臂的实际位移

    q=[-0.07 -0.0874 -2.634 1.22 0.7342 0.0162 0.53 0.0178 0]';     % 真实关节角度
    Q(9,round(tt/dt))=0;        % 真实关节角度

    R_w=0.1; b=0.15;d=0.1; % 车的参数
%% 轨迹规划
     t=dt:dt:tt; 
    
     R=4;
     omega=pi/tt;
%      X_d=R*sin(omega*t);
%      Y_d=-0.0246+R*(1-cos(omega*t));
%      Z_d=1+0*t;
     X_d=1.02479+R*sin(omega*t);
     Y_d=0.91659+R*(1-cos(omega*t));
     Z_d=1.0195+0*t;
     R_d=0+0*t;
     P_d=0+0*t;
     YY_d=0+0*t;
     XYZRPY_d=[X_d;Y_d;Z_d;R_d;P_d;Y_d];
  
%      dX_d=R*omega*cos(omega*t);
%      dY_d=R*omega*sin(omega*t);
%      dZ_d=0+0*t;
     dX_d=R*omega*cos(omega*t);
     dY_d=R*omega*sin(omega*t);
     dZ_d=0+0*t;
     dR_d=0+0*t;
     dP_d=0+0*t;
     dYY_d=0+0*t;
     dXYZRPY_d=[dX_d;dY_d;dZ_d;dR_d;dP_d;dYY_d];

%% 计算
for t=dt:dt:tt

    i=round(t/dt);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WMM 雅可比求解  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Jaco=bulid_Jaco_WMM(q,xyth(3));         % WMM的雅可比

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 运动学计算 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    R_W0=rotz(xyth(3)*180/pi);
    DH_table=build_DH_table_KinovaG3(q(3:end));
    [Ti,T] = build_T(DH_table);
    
    % 末端实际的xyzrpy
    xyz=R_W0*T(1:3,4)+[xyth(1:2);0.4];
    R=rotz(xyth(3)*180/pi)*T(1:3,1:3);
    rpy=tr2rpy(R)';
    xyzrpy=[xyz;rpy];
    
    % 末端规划的xyzrpy_d,dxyzrpy_d
    xyzrpy_d=XYZRPY_d(:,i);
    dxyzrpy_d=dXYZRPY_d(:,i);
    
    % 1 规划末端：增益，雅可比伪逆求解
    K=20;
    dq=pinv(Jaco(1:3,:))*(dxyzrpy_d(1:3)+K*(xyzrpy_d(1:3)-xyzrpy(1:3)));

%     % 2 规划末端+避奇异
%     K_xyz=[30 30 30]';
%     dq_xyz=pinv(Jaco(1:3,:))*(dxyzrpy_d(1:3)+K_xyz.*(xyzrpy_d(1:3)-xyzrpy(1:3)));
%     K_abnormal=[10 10 10 10 10 10 10]';
%     alpha=1;
%     dq_abnormal=alpha.*grad_Hs(Jaco_mani);
%     dq=dq_xyz+[0;0;dq_abnormal];
%     % 3 规划末端+避极限
%     K=50;
%     dq=pinv(Jaco(1:3,:))*(dxyzrpy_d(1:3)+K*(xyzrpy_d(1:3)-xyzrpy(1:3)));
    
%     % 4 规划末端+规划车
%     K=50;
%     dq=pinv(Jaco(1:3,:))*(dxyzrpy_d(1:3)+K*(xyzrpy_d(1:3)-xyzrpy(1:3)));
    
    % t --> t+dt时刻的运动
    Phi= build_Phi_DWMR(xyth(3));
    dxyth=Phi*dq(1:2);
    xyth=xyth+dxyth*dt;
    q=q+dq*dt;
    
    Q(:,i)=q;
    XYTH(:,i)=xyth;
    dXYTH(:,i)=dxyth;
    XYZRPY(:,i)=xyzrpy;

end

    data=[TT',Q'];
    figure(1);
    plot3(XYZRPY(1,:),XYZRPY(2,:),XYZRPY(3,:));
    title('实际轨迹');xlabel('x');ylabel('y');zlabel('z');
    figure(2);
    plot3(XYZRPY_d(1,:),XYZRPY_d(2,:),XYZRPY_d(3,:));zlim([0.8 1.2]);
    title('期望轨迹');xlabel('x');ylabel('y');zlabel('z');
    figure(3);
    subplot(2,2,1);title('x-error');plot(XYZRPY_d(1,:)-XYZRPY(1,:));title('x跟踪误差');
    subplot(2,2,2);title('y-error');plot(XYZRPY_d(2,:)-XYZRPY(2,:));title('y跟踪误差');
    subplot(2,2,3);title('z-error');plot(XYZRPY_d(3,:)-XYZRPY(3,:));title('z跟踪误差');
    