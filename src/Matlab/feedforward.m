function uf = feedforward(qd)
J=eye(length(qd));
B=eye(length(qd));
s=tf('s');
ff = J*s^2+B*s;
uf =lsim(ff)
end