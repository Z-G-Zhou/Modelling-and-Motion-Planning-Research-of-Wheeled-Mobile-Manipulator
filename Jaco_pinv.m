function Jaco_pinv = Jaco_pinv(Jaco)
%% Jaco_pinv = pinv_Jaco(Jaco)  计算雅可比矩阵非奇异值下的伪逆

    sigma_a_min=0.02;  % sigma_a_min
    k_0=0.02;     % k_0
    measure=svd(Jaco);
    sigma_a=min(measure);
    if sigma_a<=sigma_a_min
         k_d=k_0*(1-sin(pi/2*sigma_a/sigma_a_min));    
    else
        k_d=0; 
    end  
    
    Jaco_pinv = (Jaco)'*inv(Jaco*Jaco'+k_d*eye(size(Jaco*Jaco',1)));

end