function p=getPath(vchild,RRT_tree)

id=vchild.pid;
[~,nc]=size(RRT_tree);
p=[];
while(id>0)
    p=[vchild.edgeq;p];
    for i=1:nc
        if(RRT_tree(i).id==id)
            vchild=RRT_tree(i);
            id=vchild.pid;
            break;
        end
    end    
end
end

