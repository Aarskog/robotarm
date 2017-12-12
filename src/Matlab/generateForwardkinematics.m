syms q1 q2 q3 q4 q5

d1 = 0.0796;
a2 = 0.1347;
a3 = 0.0712;
d5 = 0.0918;
robotlength = d1+a2+a3+d5;


%syms s1 s2 s3 s4 s5 c1 c2 c3 c4 c5 d1 a2 a3 d5
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
c5 = cos(q5);

s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);
s5 = sin(q5);

%NEW A matrices
A1 = [s1 0 c1 0;
    -c1 0 s1 0;
    0 -1 0 d1;
    0 0 0 1];

A2 = [-s2 -c2 0 a2*s2;
    c2 -s2 0 -a2*c2;
    0 0 1 0;
    0 0 0 1];

A3 = [c3,-s3,0,-a3*c3;
    s3,c3,0,-a3*s3;
    0,0,1,0;
    0,0,0,1];

A4 = [s4 0 -c4 0;
    -c4 0 -s4 0
    0 1 0 0
    0 0 0 1];

A5 = [-s5 -c5 0 0;
    c5 -s5 0 0;
    0 0 1 d5
    0 0 0 1];

T5 = simplify(A1*A2*A3*A4*A5);