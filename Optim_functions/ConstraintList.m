classdef ConstraintList
    properties
        Name = {}
        List = {}
        LowerBound
        UpperBound
    end
    methods
        function obj = ConstraintList()

        end

        function obj = AddConstraint(obj, name, constr, lb, ub)
            obj.Name = {obj.Name{:}, name}; 
            obj.List = {obj.List{:}, constr}; 
            obj.LowerBound = [obj.LowerBound; lb]; 
            obj.UpperBound = [obj.UpperBound; ub]; 
        end

    end
end