function qdlast = getqdLast(qdarr,i,n)
    sizum = size(qdarr);
    sizum = sizum(2);
    qdlast = zeros(5,n);
    i2 = i;
    for j = 1:n
        if i2<=1
           i2 = sizum; 
        end
        i2=i2-1;
        qdlast(:,j) = qdarr(:,i2);
    end
end