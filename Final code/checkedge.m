function status=checkedge(obs,v_new);


[npoints,~]=size(v_new.edgeq);

% status == 1 no collision on the edge
% status == -1 collision on the edge

for i=1:npoints
    if(checkcollision(v_new.edgeq(i,1:2),obs)>0)
        status=-1;
        break;
    else
        status=1;
    end
end
end
