classdef VariableList
    properties
        Variable = {}
        VariableConfiguration
        Type = {}
        Symbolics = {}
        LowerBound
        UpperBound
        Init
    end
    methods
        function obj = VariableList()
            keys = [1 2 3 4 5 6]; 
            values = ["Time", "Position", "Velocity", "Acceleration", "Torque_Input", "Ground_Reaction_Force"]; 
            obj.VariableConfiguration.Type = values; 
            obj.VariableConfiguration.LowerBound = cell(1,6);
            obj.VariableConfiguration.UpperBound = cell(1,6);
            obj.VariableConfiguration.Dictionary = dictionary(values, keys); 
        end

        function obj = AddVariableToList(obj, var) 
            obj.Type = {obj.Type{:}, var.Type}; 
            obj.Symbolics = {obj.Symbolics{:}, var.Symbolic}; 
            obj.LowerBound = [obj.LowerBound; var.LowerBound]; 
            obj.UpperBound = [obj.UpperBound; var.UpperBound]; 
            obj.Init = [obj.Init; var.Init]; 
        end

        function obj = AddVariableTypes(obj, var)
            obj.Variable = {obj.Variable{:}, var};
        end

        function obj = BuildVariableList(obj, varlist, N)
            for i = 1:N
                for j = 1:numel(varlist)
                    varnew = Variable([varlist(j).Name num2str(i)], varlist(j).Size, varlist(j).LowerBound, varlist(j).UpperBound, varlist(j).Init, varlist(j).Type);
                    obj = AddVariableToList(obj, varnew); 
                end
            end
            for k = 1:numel(varlist)
                obj = AddVariableTypes(obj, varlist(k));
            end
        end

        function obj = UpdateConfiguration(obj, vartypes, lb, ub)
            dic = obj.VariableConfiguration.Dictionary;
            for i = 1:numel(vartypes)
                idx = dic(vartypes{i}); 
                obj.VariableConfiguration.LowerBound{idx} = lb{i}; 
                obj.VariableConfiguration.UpperBound{idx} = ub{i}; 
            end
            
        end

        function x = ExtractState(obj, state)
            q = obj.Symbolics(find(string(obj.Type) == "Position"));
            dq = obj.Symbolics(find(string(obj.Type) == "Velocity"));

            if state == "Initial"
                x = [q{1};dq{1}];
            elseif state == "Final"
                x = [q{end};dq{end}];
            else
                x = [q;dq]; 
            end

        end

        function t = ExtractTime(obj)
            t_full = obj.Symbolics(find(string(obj.Type) == "Time"));
            t = t_full{1}; 
        end

        function tau = ExtractTorque(obj)
            tau = obj.Symbolics(find(string(obj.Type) == "Torque_Input"));
     
        end

        function recovered_vars = ExtractIndices(obj, optvector)
            
            sizes = cellfun(@(x) numel(x), obj.Symbolics);
            
            % Reconstruct
            values = cell(1, numel(obj.Type));
            idx = 1;
            for i = 1:numel(obj.Type)
                len = sizes(i);
                values{i} = optvector(idx:idx+len-1);
                idx = idx + len;
            end

            % recovered_vars = struct('Type', obj.Type, 'Value', values);
            recovered_vars.Type = obj.Type; 
            recovered_vars.Values = values; 
        end

    end
end