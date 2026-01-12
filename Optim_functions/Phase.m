classdef Phase
    properties
        DynamicModel
        Name
        Collocation
        InitialConfiguration
        ContactConfiguration
        Transition
        VariableList
        ConstraintList
        Cost
    end
    methods
        function obj = Phase(name, contactconfig, N)
            obj.Name = name; 
            obj.Collocation.Points = N; 
            obj.ContactConfiguration = contactconfig; 
            obj.ConstraintList = ConstraintList();
            obj.VariableList = VariableList(); 
             
        end

        function obj = ConfigurePhase(obj, contactconfig, transitiontype, transitioncontacts, collocation, initconfig, cost)
            obj.ContactConfiguration = contactconfig; 
            obj.Transition.Type = transitiontype; 
            obj.Transition.Contacts = transitioncontacts; 
            obj.Collocation.Scheme = collocation; 
            
            if nargin < 6
            else
                obj.InitialConfiguration = initconfig; 
            end
            
        end

        function obj = BuildPhase(obj)
            if size(obj.DynamicModel.Spatial_Robot.Xtree{1}) == [6,6]
                obj = PhaseBuilder(obj); 
            else
                obj = PhaseBuilder2D(obj); 
            end
        end


        function obj = AddBoundsToPhase(obj, vartypes, lb, ub)
            obj.VariableList = obj.VariableList.UpdateConfiguration(vartypes, lb, ub); 
        end

        function obj = AddInitialConstraint(obj, vartype, idx, lb, ub)

        end

        function obj = AddFinalConstraint(obj, vartype, idx, lb, ub)
        end

        function obj = AddContinuousConstraint(obj, vartype, idx, lb, ub)
        end
        
    end
end