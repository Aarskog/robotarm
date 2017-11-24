function dk = directKinematics(T5,q)
    T5t = getTransformationMatrix(T5,q);
    
    r21 = T5t(2,1); 
    r11 = T5t(1,1);
    r31 = T5t(3,1);
    r32 = T5t(3,2);
    r33 = T5t(3,3);
    thetaz = atan2(r21,r11);
    thetay = atan2(-r31,-sqrt(r32^2+r33^2));
    thetax = atan2(r32,r33);
    p5 = T5t(1:3,4);
    %Direct kinematics
    dk = [p5;thetaz;thetay;thetax];
end