function [] = collisionIF(X,t)

dis = 0;
for i = 1:size(X,2)
    for j = i:size(X,2)
        if i~=j
        dis = norm(X(1:3,i) - X(1:3,j)); 
        end
        if(dis<=0.3)
           disp('collision occur! distance is ');
           disp(dis);
            disp('time step is ');
            disp(t);
        end
    end
end
