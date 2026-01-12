
function obj = Collocation(obj, d, collocationscheme)
    import casadi.*
    
    % Access Variable Configurations
        dic = obj.VariableList.VariableConfiguration.Dictionary; 
        idxT = dic("Time"); idxP = dic("Position"); idxV = dic("Velocity"); idxA = dic("Acceleration"); idxTI = dic("Torque_Input");
        varlow = obj.VariableList.VariableConfiguration.LowerBound;
        varup = obj.VariableList.VariableConfiguration.UpperBound;
    
        % Initial variables
        NB = obj.DynamicModel.Spatial_Robot.NB; 
        T = Variable('T',[1 1]); T.Type = "Time"; 
        Q = Variable('Q', [NB,1]);   Q.Type = "Position"; 
        DQ = Variable('DQ', [NB,1]); DQ.Type = "Velocity"; 
        DDQ = Variable('DDQ', [NB,1]); DDQ.Type = "Acceleration"; 

        if NB > 6
            TAU = Variable('TAU', [NB-6,1]); TAU.Type = "Torque_Input"; 
        else
            TAU = Variable('TAU', [NB,1]); TAU.Type = "Torque_Input"; 
        end
    
        % Add Bounds
        T = AddBounds(T, varlow{idxT}, varup{idxT});
        Q = AddBounds(Q,varlow{idxP}, varup{idxP}); 
        DQ = AddBounds(DQ,varlow{idxV}, varup{idxV}); 
        DDQ = AddBounds(DDQ,varlow{idxA}, varup{idxA}); 
        TAU = AddBounds(TAU,varlow{idxTI}, varup{idxTI});
    
    
    N = obj.Collocation.Points;
    NB = obj.DynamicModel.Spatial_Robot.NB; 
    dt = 1/(N); 

    % Get collocation points
    coll_points = collocation_points(d, collocationscheme);
    
    % Collocation linear maps
    [C,D,B] = collocation_coeff(coll_points);
    

    % Check if phase is in contact
    if isempty(obj.ContactConfiguration)
                    % obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TAU], 1); 
                    obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ], 1); 
                    % obj.VariableList = obj.VariableList.BuildVariableList([Q;DQ], N*d+1); 
                    CollocationVariables = []; 
                    for k = 1:d
                       CollocationVariables = [CollocationVariables;Q;DQ;DDQ];
                    end

                    obj.VariableList = obj.VariableList.BuildVariableList([T;CollocationVariables;TAU], N);
                    % obj.VariableList = obj.VariableList.BuildVariableList([TAU], N);
                    % obj.VariableList = obj.VariableList.BuildVariableList([T], N+1);
                    % obj.VariableList = obj.VariableList.BuildVariableList([DDQ], N*d);

                else
                    NF = numel(obj.ContactConfiguration); % number of contact forces
                    TotalGRFVariables = []; 
    
                    for i = 1:NF
                        GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-1000;-1000;0], [1000; 1000; 1000], [50;50;100]);
                        GRF.Name = ['GRF_' num2str(obj.ContactConfiguration{i})]; 
                        GRF.Type = "Ground_Reaction_Force"; 
                        TotalGRFVariables = [TotalGRFVariables;GRF];
                    end
                        CollocationVariables = []; 
                    for k = 1:d
                       CollocationVariables = [CollocationVariables;Q;DQ;DDQ;TotalGRFVariables];
                    end
                     
                    obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ], 1); 
                    % obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TotalGRFVariables;TAU], 1); 
                    obj.VariableList = obj.VariableList.BuildVariableList([T;CollocationVariables;TAU], N);
                    % obj.VariableList = obj.VariableList.BuildVariableList([Q;DQ], N*d+1); 
                    % obj.VariableList = obj.VariableList.BuildVariableList([TAU], N);
                    % obj.VariableList = obj.VariableList.BuildVariableList([T], N+1);
                    % obj.VariableList = obj.VariableList.BuildVariableList([DDQ;TotalGRFVariables], N*d);
    
    end

    t = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Time"));
    q = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Position"));
    dq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Velocity"));
    ddq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Acceleration"));
    tau = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Torque_Input"));
    
    % Configure constraints
    if isempty(obj.ContactConfiguration)
            NF = 0; 
            f = obj.DynamicModel.Dynamics;
    
            % Extract variables
            x = [q{1};dq{1}];
            % Apply initial constraints
            if ~isempty(obj.InitialConfiguration)
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Initial Configuration", x - obj.InitialConfiguration, zeros(2*NB,1), zeros(2*NB,1));
            end
    
            % obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1}),zeros(NB,1),zeros(NB,1));
    
            % Inital cost
            J = 0;
    
            idx = 0; 
            for i = 1:N
                % Indices
                counter = idx+1; 
                idx = i*d; 
  
                % Extract collocation and mesh points
                X_i = [q{counter};dq{counter}]; 
                X_c = [q(counter+1:idx+1);dq(counter+1:idx+1)];
                X_c = reshape(vertcat(X_c{:}),[NB*2, d]);
                DDQ_c = ddq(counter:idx); 
                DX_c = [dq(counter+1:idx+1);DDQ_c];
                DDQ_c = reshape(vertcat(DDQ_c{:}), [NB, d]);
                TAU_c = tau{i};
                
                % Evaluate dynamics
                ode = f(X_c,DDQ_c,TAU_c);
               
                % Collocation
                Z = [X_i X_c]; 
    
                dZ = reshape(vertcat(DX_c{:}),NB*2,d); 
                Pidot = Z*C;
    
                % coll_con = Pidot - dt*t{i}*dZ;
                coll_con = Pidot - (t{i}/N)*dZ;
                % coll_con = coll_con/10; 

                obj = FootConstraints(obj, X_c); 
    
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Time_Continuity",t{i+1} - t{i} , 0,0);
                
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",vertcat(ode{:}),zeros(d*NB,1),zeros(d*NB,1));
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation",vertcat(coll_con{:}),zeros(d*2*NB,1),zeros(d*2*NB,1));

                IntCost = 0; 
                for j = 1:d
                    IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,j)); 
                % IntCost = Cost(obj.Cost.Type, TAU_c, X_c(:,1));
                % IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,2));
                % IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,3));
                end
                IntCostArray = IntCost*ones(1,d);
                
                % Evaluate quadrature cost
                % J = J + IntCostArray*B*dt;
                J = J + IntCostArray*B*(t{i}/N);
                % J = IntCost;
            end
            % obj.Cost.Symbolic = J*t{1}; 
            % obj.Cost.Symbolic = sumsqr(vertcat(tau{:}));
            obj.Cost.Symbolic = J;
            

    else
            % Extract dynamic functions
            obj.DynamicModel = obj.DynamicModel.SpecifyContact(obj.ContactConfiguration);
            f = obj.DynamicModel.ConstrainedDynamics;
            h = obj.DynamicModel.Holonomic;

            % Extract ground reaction forces
            NH = 3*NF; % number of holonmic constraints
            grf_values = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Ground_Reaction_Force"));
        
            % Extract initial variables
            x = [q{1};dq{1}];
            grf = {}; 
            for j = 1:N*d
                start_idx = (j-1) * NF + 1;
                end_idx = start_idx + NF - 1; 
                grf_prelim = grf_values(start_idx:end_idx); 
                grf_conc = []; 
                for k = 1:NF
                    grf_conc = [grf_conc; grf_prelim{k}];

                    grf{j} = grf_conc;
                end
            end

            % Add initial constraints   
            if ~isempty(obj.InitialConfiguration)
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Initial Configuration", x - obj.InitialConfiguration, zeros(2*NB,1), zeros(2*NB,1));
            end

            % obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1},grf{1}),zeros(NB,1),zeros(NB,1));
            % obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x,ddq{1}),zeros(NH,1),zeros(NH,1));

            % Inital cost
            J = 0;
            idx = 0;

            for i = 1:N
                % Indices
                counter = idx+1; 
                idx = i*d; 
  
                % Extract collocation and mesh points
                X_i = [q{counter};dq{counter}]; 
                X_c = [q(counter+1:idx+1);dq(counter+1:idx+1)];
                X_c = reshape(vertcat(X_c{:}),[NB*2, d]);
                DDQ_c = ddq(counter:idx); 
                DX_c = [dq(counter+1:idx+1);DDQ_c];
                DDQ_c = reshape(vertcat(DDQ_c{:}), [NB, d]);
                GRF_c = grf(counter:idx);
                GRF_c = reshape(vertcat(GRF_c{:}), [NF*3, d]);
                TAU_c = tau{i};
                
                % Evaluate dynamics and holonomic constraint
                ode = f(X_c,DDQ_c,TAU_c,GRF_c); 
                holonomic = h(X_c,DDQ_c);

                % Collocation
                Z = [X_i X_c]; 
    
                dZ = reshape(vertcat(DX_c{:}),NB*2,d); 
                Pidot = Z*C;
    
                % coll_con = Pidot - dt*t{i}*dZ;
                coll_con = Pidot - (t{i}/N)*dZ;
                % coll_con = coll_con/10; 

                % Apply constraints
                obj = FootConstraints(obj, X_c); 

                obj.ConstraintList = obj.ConstraintList.AddConstraint("Time_Continuity",t{i+1} - t{i} , 0,0);
                
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",vertcat(ode{:}),zeros(d*NB,1),zeros(d*NB,1));

                obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",vertcat(holonomic{:}),zeros(d*NH,1),zeros(d*NH,1));
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation",vertcat(coll_con{:}),zeros(d*2*NB,1),zeros(d*2*NB,1));

                IntCost = 0; 
                for j = 1:d
                    IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,j)); 
                    % IntCost = Cost(obj.Cost.Type, TAU_c, X_c(:,1));
                    % IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,2));
                    % IntCost = IntCost + Cost(obj.Cost.Type, TAU_c, X_c(:,3)); 
                end
                IntCostArray = IntCost*ones(1,d);
                
                % Evaluate quadrature cost
                % J = J + IntCostArray*B*dt;
                J = J + IntCostArray*B*(t{i}/N);
                % J = IntCost;
               

            end
            % obj.Cost.Symbolic = J*t{1}; 
            obj.Cost.Symbolic = J; 
            % obj.Cost.Symbolic = sumsqr(vertcat(tau{:}));
           
    
    end
    
end
