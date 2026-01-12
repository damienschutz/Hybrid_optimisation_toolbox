classdef DynamicModel
    properties
        Name
        Mass
        Spatial_Robot      % From featherstones method
        CasADi_Symbols
        Contacts
        Dynamics
        ConstrainedDynamics
        Holonomic
        Impact
    end
    methods
        function obj = DynamicModel(model)
            obj.Spatial_Robot = model();  
            if size(obj.Spatial_Robot.Xtree{1}) == [6, 6] 
                obj = CreateCasadiModel(obj, model); 
            else
                obj = CreateCasadiModel2D(obj, model); 
            end
        end

        function obj = SpecifyContact(obj, input)
            obj = AddContactDynamics(obj,input);
        end

        function obj = SpecifyImpact(obj, input)
            if size(obj.Spatial_Robot.Xtree{1}) == [6, 6] 
                obj = AddImpact(obj, input); 
            else
                obj = AddImpact2D(obj, input); 
            end
        end

        function footfunc = SpecifyFoot(obj, input)
            idx = obj.Contacts.Dictionary(string(input)); 
            footfunc = obj.Contacts.FeetPositions{idx};
        end

        function obj = AddSpring(obj, k, idx)
            ode = obj.CasADi_Symbols.ode_sym; 
            q = obj.CasADi_Symbols.q; 

            ode(idx) = ode(idx) + k*q(idx); 
            obj.CasADi_Symbols.ode_sym = ode; 
            obj = UpdateCasadiModel(obj); 
        end

        function obj = AddSpringDamper(obj, k, b, idx)
            ode = obj.CasADi_Symbols.ode_sym; 
            q = obj.CasADi_Symbols.q; 
            dq = obj.CasADi_Symbols.dq; 

            ode(idx) = ode(idx) + k*q(idx) + b*dq(idx); 
            obj.CasADi_Symbols.ode_sym = ode; 
            obj = UpdateCasadiModel(obj); 
        end


        function obj = AddSpringDamperBounds(obj, k, b, idx, limit)
            ode = obj.CasADi_Symbols.ode_sym; 
            q = obj.CasADi_Symbols.q; 
            dq = obj.CasADi_Symbols.dq; 

            ode(idx) = ode(idx) + k*(q(idx)-limit) + b*dq(idx); 
            obj.CasADi_Symbols.ode_sym = ode; 
            obj = UpdateCasadiModel(obj); 
        end

        function obj = UpdateCasadiModel(obj)
            import casadi.*
            ContinuousDynamics = obj.CasADi_Symbols.ode_sym; 
            q = obj.CasADi_Symbols.q;
            dq = obj.CasADi_Symbols.dq;
            x = [q;dq];
            ddq = obj.CasADi_Symbols.ddq;
            u = obj.CasADi_Symbols.u;
            
            obj.Dynamics = Function('ContinuousDynamics', {x, ddq, u},{ContinuousDynamics},{'x','ddq','u'},{'ContinuousDynamics'});
        end


    end
end