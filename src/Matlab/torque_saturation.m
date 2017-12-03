function u = torque_saturation(u,maxtorque)
    for i=1:length(u)
       if u(i)>maxtorque
            u(i)=maxtorque;
       end
       if u(i)<-maxtorque
            u(i)=-maxtorque;
       end
    end
end