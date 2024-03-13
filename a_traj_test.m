T=350;
dt=0.01;

XYZRPY_d=zeros(T/dt+1,6);         % 规划末端位置
XYTH_d=zeros(T/dt+1,3);             % 规划车位置

for t=0:dt:T
    [xyzrpy_d,dxyzwxyz_d,xyth_d,dxyth_d,q_0] = traj_plan_scene(t,T);
    XYZRPY_d(round(t/dt)+1,:)=xyzrpy_d';         % 规划末端位置
    XYTH_d(round(t/dt)+1,:)=xyth_d';             % 规划车位置
end

distance=zeros(1,size(XYZRPY_d,1));
for i=1:size(XYZRPY_d,1)
    distance(i)=sqrt((XYZRPY_d(i,1)-XYTH_d(i,1))^2+(XYZRPY_d(i,2)-XYTH_d(i,2))^2);
end

t=0:0.01:350;

figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
% plot3(XYZRPY_d(:,1),XYZRPY_d(:,2),XYZRPY_d(:,3),"-.");
plot(XYZRPY_d(:,1),XYZRPY_d(:,2),"-");hold on;
plot(XYTH_d(:,1),XYTH_d(:,2),'--');

xlabel("x(m)");
ylabel("y(m)");
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

legend('末端规划轨迹','小车规划轨迹');
legend('FontName', '宋体', 'FontSize', 10.5);



figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(t,distance);

xlabel("\it\fontname{Times New Roman}t\rm\fontname{Times New Roman}(s)");
ylabel("\fontname{宋体}距离\fontname{Times New Roman}(m)");
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);

