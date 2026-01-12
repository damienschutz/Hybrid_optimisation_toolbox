function obj = FootConstraintsImpact2D(obj, x)
    dic = obj.DynamicModel.Contacts.Dictionary;
    keysToKeep = string(obj.Transition.Contacts);


    for i = 1:length(keysToKeep)
        idx = dic(keysToKeep{i}); 
        foot_func = obj.DynamicModel.Contacts.FeetPositions{idx}; 

        footvel_func = obj.DynamicModel.Contacts.FeetVelocities{idx}; 

        foot_pos = foot_func(x); 

        obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot_Impact_Position', foot_pos(2), 0, 0);  
     end

end