function DH_table_KinovaG3 = build_DH_table_KinovaG3(q)
%  DH_table_KinovaG3 = build_DH_table_KinovaG3(q)  生成q对应时刻机械臂（改进）DH参数表
%  DH_table以矩阵形式表示，对于n自由度的机械臂：
%  * 列标从1-4分别为连杆扭转角（rad）、连杆长度、连杆偏距、关节角（rad）
%  * 行标i为i-1系到i系变换矩阵T所需的参数
%  注明：最后行是末连杆系到End-effector系的“等效DH参数”
%

%% 主程序
al_E=0;a_E=0;d_E=0.2374;q_E=0;   % 末端系到End系等效DH参数设置

DH_table=[
        0,   0,   0.2848;
     pi/2,   0,   0.0118;
    -pi/2,   0,   0.4208;
     pi/2,   0,   0.0128;
    -pi/2,   0,   0.3143;
     pi/2,   0,        0;
    -pi/2,   0,        0;
     al_E, a_E,      d_E];

%% 维数不匹配，输出错误信息
[m,~]=size(DH_table);
if m~=(length(q)+1)
    error('length of q is not correct! ');
end
[~,nq]=size(q);
if nq==length(q)
    q=q';
end
q=[q;q_E];
%% 返回最终结果
DH_table_KinovaG3 = [DH_table,q];

end

