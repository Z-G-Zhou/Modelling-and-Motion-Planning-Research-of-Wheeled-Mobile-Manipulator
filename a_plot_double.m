clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 双轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 t=0:0.001:100;

%% 末端轨迹
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(XYZRPY_d(:,1),XYZRPY_d(:,2),"-");hold on;
plot(XYZRPY_WE(:,1),XYZRPY_WE(:,2),"-.");

xlabel('x(m)');
ylabel('y(m)');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

legend('规划轨迹','实际轨迹');
legend('FontName', '宋体', 'FontSize', 10.5);


%% 末端误差
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(t,1000.*(XYZRPY_d(:,1)-XYZRPY_WE(:,1)),'-');hold on;
plot(t,1000.*(XYZRPY_d(:,2)-XYZRPY_WE(:,2)),'--');hold on;
plot(t,1000.*(XYZRPY_d(:,3)-XYZRPY_WE(:,3)),'-.');hold on;

xlabel('\it\fontname{Times New Roman}t\rm\fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}误差\fontname{Times New Roman}(10^{-3}m)');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

legend('\fontname{Times New Roman}x\fontname{宋体}误差','\fontname{Times New Roman}y\fontname{宋体}误差','\fontname{Times New Roman}z\fontname{宋体}误差');
legend('FontName', 'Times New Roman', 'FontSize', 10.5);


%% 车轨迹
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(XYTH_d(:,1),XYTH_d(:,2),'-');hold on;
plot(XYZRPY_W0(:,1),XYZRPY_W0(:,2),'--');

xlabel('x(m)');
ylabel('y(m)');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

legend('规划轨迹','实际轨迹');
legend('FontName', '宋体', 'FontSize', 10.5);


%% 车误差
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(t,1000.*(XYTH_d(:,1)-XYZRPY_W0(:,1)),'-');hold on;
plot(t,1000.*(XYTH_d(:,2)-XYZRPY_W0(:,2)),'--');hold on;

xlabel('\it\fontname{Times New Roman}t\rm\fontname{Times New Roman}(s)');
ylabel('\fontname{宋体}误差\fontname{Times New Roman}(10^{-3}m)');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

legend('\fontname{Times New Roman}x\fontname{宋体}误差','\fontname{Times New Roman}y\fontname{宋体}误差');
legend('FontName', 'Times New Roman', 'FontSize', 10.5);
