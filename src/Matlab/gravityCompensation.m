function gravitycomp = gravityCompensation(q)
    g = -9.81;
    
    %Link Mass
    m1 = 0.126+0.05;
    m2 = 0.01+0.03+0.126;
    m3 = 0.126+0.01;
    m4 = 0.126+0.01*2; %Multiplied for better results
    m5 = 0.126+0.01*3; %Multiplied for better results
    %m5 = 0.126+0.01*2;
    
    % Link lengths
    l1 = 0.0506+0.01;
    l2 = 0.0206+0.0635+0.0506;
    l3 = 0.0206+0.0506;
    l4 = 0.0206+0.0506;
    l5 = 0.0506/2+0.01;
    
    
    s234 = sin(q(2) +q(3) + q(4));
    s23 = sin(q(2) + q(3));
    s2 = sin(q(2));
    
    l23 = m3*g*(l2*s2 + l3/2*s23);
    l24 = m4*g*(l2*s2 + l3*s23 + l4/2*s234);
    l25 = m5*g*(l2*s2 + l3*s23 + (l4+l5/2)*s234);
    
    l34 = m4*g*(l3*s23 + l4/2*s234);
    l35 = m5*g*(l3*s23 + (l4+l5/2)*s234);
    
    l45 = m5*g*(l4+l5/2)*s234;
    
    g1 = 0;
    g2 = m2*g*l2/2*s2 + l23 + l24 + l25;
    g3 = m3*g*l3/2*s23 + l34 +l35;
    g4 = m4*g*l3/2*s234 + l45;
    g5 = 0;
    
    gravitycomp = [g1;g2;g3;g4;g5];
    
end