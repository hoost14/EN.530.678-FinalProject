function [v_new, RRT_tree]=extendblossom(RRT_tree,obs,v_near,qrand,motionPrimitiveCommandArray,delta)

global addedVert
global cntId
[nPrimitives ~]=size(motionPrimitiveCommandArray);

% ode parameters
tSpan = [0 delta]; % solve from t=0 to t=delta
q0 = v_near.pose;




for i=1:nPrimitives
    
    
    %     generate the i-th motion primitive
    v     = motionPrimitiveCommandArray(i,1);
    omega = motionPrimitiveCommandArray(i,2);
    [T, Q] = ode45(@(t,q) EdumipKinematics( t, q, v ,omega ),tSpan,q0); % solve ODE
    
    
    cntId=cntId+1;
    
    v_new.id=cntId;
    v_new.pose=Q(end,:)';
    v_new.edgeq=Q(:,:);
    v_new.edgeu=[v omega];
    v_new.pid=v_near.id;

    
    % check collision
    status=checkedge(obs,v_new);
    if(status == -1)
        disp('collision')
        cntId=cntId-1;
      
        continue
    end
     



    flag_regression=regression(v_near,v_new,RRT_tree);
    if(flag_regression == 1)
        cntId=cntId-1;
        disp('regression')
        continue
    end
    
    
    
    addedVert=addedVert+1;
    RRT_tree(addedVert).id=v_new.id;
    RRT_tree(addedVert).pose=v_new.pose;
    RRT_tree(addedVert).edgeq=v_new.edgeq;
    RRT_tree(addedVert).edgeu=v_new.edgeu;
    RRT_tree(addedVert).pid=v_new.pid;
    
    
    figure(1),plot(RRT_tree(addedVert).edgeq(:,1),RRT_tree(addedVert).edgeq(:,2));
    
    
    
end


v_new=NearestTree(RRT_tree,qrand);

end

