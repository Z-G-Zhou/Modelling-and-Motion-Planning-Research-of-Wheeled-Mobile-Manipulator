function Phi= build_Phi_DWMR(th)
%% Phi= build_Phi_DWMR(th) 获得双轮移动车的速度传递矩阵
%   给定参数：车轮半径，车轮距，基座安装前向偏置，当前车转角
%   [v_x,v_y,omega]'=Phi*[w_l,w_r]'

%%%%%%%%%%%%%%%%%%%%%% 车参数 %%%%%%%%%%%%%%%%%%%%%%
R_w=0.1; % 轮半径       
b=0.275; % 两轮距离的一半      
d=0.1;    %基座安装前置距离  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Phi= R_w/2.0/b*...
          [b*cos(th)+d*sin(th),b*cos(th)-d*sin(th);...
           b*sin(th)-d*cos(th),b*sin(th)+d*cos(th);...
                   -1,                1         ];
end

