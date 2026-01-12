function obj = FootConstraintsImpact(obj, x)
    dic = obj.DynamicModel.Contacts.Dictionary;
    keysToKeep = string(obj.Transition.Contacts);


    for i = 1:length(keysToKeep)
        idx = dic(keysToKeep{i}); 
        foot_func = obj.DynamicModel.Contacts.FeetPositions{idx}; 

        footvel_func = obj.DynamicModel.Contacts.FeetVelocities{idx}; 

        foot_pos = foot_func(x); 
        foot_vel = footvel_func(x);

        obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot_Impact_Position', foot_pos(3), 0, 0);  
        
        % obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot_Impact_Forward_speed', foot_vel(1), -0.1, 0.1); 
        % obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot_Impact_Lateral_Speed', foot_vel(2), -0.1, 0.1); 
        % obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot_Impact_Vertical_Speed', foot_vel(3), -inf, -0.1);
    end

end