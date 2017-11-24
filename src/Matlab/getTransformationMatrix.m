function Tm = getTransformationMatrix(T5,q)
    syms q1 q2 q3 q4 q5;
    Tm = double(subs(T5,[q1 q2 q3 q4 q5],[q(1) q(2) q(3) q(4) q(5)]));
end