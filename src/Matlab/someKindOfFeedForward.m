function qd = someKindOfFeedForward(qarr,i)
n = size(qarr);
n=n(2);
forwardlength = 4;

if (i+forwardlength)<n
    qd = qarr(:,i)+qarr(:,i:(i+forwardlength))*ones(5,1)/forwardlength^4;
else
    qd = qarr(:,i);
end   

end