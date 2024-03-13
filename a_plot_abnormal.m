clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 奇异位形 Hs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% t=0:0.001:100;
[m,n]=size(Q);
Hs=zeros(m,1);
for i=1:m
    DH_table=build_DH_table_KinovaG3(Q(i,3:end));
    Jaco=build_Jaco_manipulator(DH_table);
    Hs(i)=det(Jaco*Jaco');
end
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6])  % 设置图的大小为 8x6 厘米
plot(t,1000.*Hs);
xlabel('\it\fontname{Times New Roman}t\rm\fontname{Times New Roman}(s)');
ylabel('\it\fontname{Times New Roman}H_s\rm\fontname{Times New Roman}(\times10^{-3})')
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);