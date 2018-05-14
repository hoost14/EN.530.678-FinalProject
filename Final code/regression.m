function flag=regression(q_near,v_new,RRT_tree)

% regression testing.



distance=norm(q_near.pose(1:2)-v_new.pose(1:2));

flag=0;
vchild=q_near;
id=vchild.pid;
[~, nc]=size(RRT_tree);
while(id>0)    
    if(norm(v_new.pose(1:2)-vchild.pose(1:2))<distance)
    flag=1
    break;
    else
    flag=0;
    end
    for i=1:nc
        if(RRT_tree(i).id==id)
            vchild=RRT_tree(i);
            id=vchild.pid;
            break;
        end
    end    
end
end
