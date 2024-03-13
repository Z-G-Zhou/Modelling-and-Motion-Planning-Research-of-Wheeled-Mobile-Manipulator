function Jaco = build_Jaco_WMM(q,th)
% function Jaco = build_Jaco_WMM(q,th)  得到WMM雅可比矩阵
% q为关节角度，th为车的转角

    J_b= build_Jaco_DWMR_base(th);
    DH_table_KinovaG3=build_DH_table_KinovaG3(q(3:end)); % 机械臂DH参数表
    J_m0 = build_Jaco_manipulator(DH_table_KinovaG3);
    R_W0=rotz(th*180/pi);
    J_m=[R_W0,zeros(3,3);zeros(3,3),R_W0 ] * J_m0;
    Jaco=[J_b,J_m];

end

