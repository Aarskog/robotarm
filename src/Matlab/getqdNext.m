function qdnext = getqdNext(qdarr,i,n)
    sizum = size(qdarr);
    sizum = sizum(2);
    qdnext = zeros(5,n);
    i2 = i;
    for j = 1:n
        if i2>=sizum
           i2 = 1; 
        end
        i2=i2+1;
        qdnext(:,j) = qdarr(:,i2);
    end
end