function u=getControls(vchild,RRT_tree)

id=vchild.pid;
[~, nc]=size(RRT_tree);
u=[];
while(id>0)
    u=[vchild.edgeu;u];
    for i=1:nc
        if(RRT_tree(i).id==id)
            vchild=RRT_tree(i);
            id=vchild.pid;
            break;
        end
    end    
end
end
