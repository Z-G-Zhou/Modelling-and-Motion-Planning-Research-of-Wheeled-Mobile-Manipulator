clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 关节限位 Hl %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% t=0:0.001:100;
qlim = build_qlim_KinovaG3;
[m,n]=size(Q);
Hl=zeros(m,1);
gamma=[1 1 1 1 1 1 1];
for i=1:m
    for j=3:9
        Hl(i)=Hl(i)+1./gamma(j-2).*(qlim(2,j-2)-qlim(1,j-2))^2 ...
                ./(qlim(2,j-2)-Q(i,j))./(Q(i,j)-qlim(1,j-2));
    end
end
figure('Units', 'centimeters', 'Position', [0, 0, 8, 6]);  % 设置图的大小为 8x6 厘米

plot(t,Hl);

xlabel('\it\fontname{Times New Roman}t\rm\fontname{Times New Roman}(s)');
ylabel('\it\fontname{Times New Roman}H_l')
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10.5);
