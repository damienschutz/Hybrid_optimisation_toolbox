function obj = FootConstraints2D(obj, x)
    dic = obj.DynamicModel.Contacts.Dictionary;
    keysToExclude = string(obj.ContactConfiguration);

    allKeys = keys(dic); 
    
    % Find keys to keep (all keys except those to exclude)
    keysToKeep = setdiff(allKeys, keysToExclude);

    for i = 1:length(keysToKeep)
        idx = dic(keysToKeep{i}); 
        foot_func = obj.DynamicModel.Contacts.FeetPositions{idx}; 

        foot_pos = foot_func(x); 

        obj.ConstraintList = obj.ConstraintList.AddConstraint('Foot Height',foot_pos(2), 0, inf); 
    end

end