function J_b= build_Jaco_DWMR_base(th)
%% Jaco= build_Jaco_DWMR_base(th) 获得双轮移动车的雅可比

Phi= build_Phi_DWMR(th);
Jb=   [1,0,0;
       0,1,0;
       0,0,0;
       0,0,0;
       0,0,0;
       0,0,1]; 
   
J_b=Jb*Phi;
end

