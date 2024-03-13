function qlim_KinovaG3 = build_qlim_KinovaG3
%  qlim_KinovaG3 = build_qlim_KinovaG3  生成机械臂KinovaG3每个关节角的关节限位
%  列表为第i个关节角，行标1代表范围最小值，行标2代表范围最大值。

qlim_KinovaG3=[  
   -1000  -126  -1000  -147   -1000   -117    -1000;
    1000   126   1000   147    1000    117     1000] ;

qlim_KinovaG3=qlim_KinovaG3*pi/180;
end

