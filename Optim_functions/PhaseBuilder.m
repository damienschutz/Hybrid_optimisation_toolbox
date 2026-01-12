function obj = PhaseBuilder(obj)

    N = obj.Collocation.Points; 
    
    % Access Variable Configurations
    dic = obj.VariableList.VariableConfiguration.Dictionary; 
    idxT = dic("Time"); idxP = dic("Position"); idxV = dic("Velocity"); idxA = dic("Acceleration"); idxTI = dic("Torque_Input"); idxGRF = dic("Ground_Reaction_Force");
    varlow = obj.VariableList.VariableConfiguration.LowerBound;
    varup = obj.VariableList.VariableConfiguration.UpperBound;
    
    % Initial variables
    NB = obj.DynamicModel.Spatial_Robot.NB; 
    T = Variable('T',[1 1]); T.Type = "Time"; T.Init = 0.2; 
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

    % Build variable list
    % Check collocation scheme
    scheme = obj.Collocation.Scheme; 

    % Unscaled time step
    dt = 1/(N-1); 

    switch scheme
        case 'Trapezoidal'
            % Check if phase is in contact
            if isempty(obj.ContactConfiguration)
                obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TAU], N); 
            else
            
                NF = numel(obj.ContactConfiguration); % number of contact forces
                TotalGRFVariables = []; 
        
                for i = 1:NF
                    GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-1000;-1000;0], [1000;1000;1000], [200;200;200]);
                    % GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-1000;-1000;0]);
                    GRF.Name = ['GRF_' num2str(obj.ContactConfiguration{i})]; 
                    GRF.Type = "Ground_Reaction_Force"; 
                    GRF = AddBounds(GRF,varlow{idxGRF}, varup{idxGRF});
                    TotalGRFVariables = [TotalGRFVariables;GRF];
                end
        
                obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TAU;TotalGRFVariables], N); 
        
            end

            % Extract variables
            t = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Time"));
            q = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Position"));
            dq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Velocity"));
            ddq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Acceleration"));
            tau = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Torque_Input"));
            
        
            % Configure Constraints
            if isempty(obj.ContactConfiguration)
                NF = 0; 
                f = obj.DynamicModel.Dynamics;
        
                x = [q{1};dq{1}]; 
        
                % Apply initial constraints
                if ~isempty(obj.InitialConfiguration)
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Initial Configuration", x - obj.InitialConfiguration, zeros(2*NB,1), zeros(2*NB,1));
                end
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1}),zeros(NB,1),zeros(NB,1));
        
                % Inital cost
                J = 0; 
        
                % Loop to apply the rest of the equality and collocation
                % constraints
                for i = 1:N-1 
                    % Extract variables 
                    x_curr = [q{i}; dq{i}]; 
                    x_next = [q{i+1};dq{i+1}]; 
                    dx_curr = [dq{i};ddq{i}];
                    dx_next = [dq{i+1};ddq{i+1}]; 
                    
                    % Equality constraint
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x_next,ddq{i+1},tau{i+1}),zeros(NB,1),zeros(NB,1));
         
                    % Trapezoidal collocation
                    % x_eval = x_curr + t{i} * dt/2 * (dx_curr + dx_next); 
                    x_eval = x_curr + (t{i}/(N-1))/2 * (dx_curr + dx_next); 
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation",x_next - x_eval,zeros(NB*2,1),zeros(NB*2,1));

                    % Time continuity 
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity", t{i+1} - t{i}, 0, 0); 
                    
                    % Foot clearance
                    obj = FootConstraints(obj, x_next);

                    % Cost
                    % J = J + (Cost(obj.Cost.Type,tau{i},x_curr)+Cost(obj.Cost.Type,tau{i+1},x_next))*dt/2;
                    J = J + (Cost(obj.Cost.Type,tau{i},x_curr)+Cost(obj.Cost.Type,tau{i+1},x_next))*(t{i}/(N-1))/2;
            
                end
            
                % obj.Cost.Symbolic = t{1}*J; 
                obj.Cost.Symbolic = J;
        
            else
            
                % Extract dynamic functions
                obj.DynamicModel = obj.DynamicModel.SpecifyContact(obj.ContactConfiguration);
                f = obj.DynamicModel.ConstrainedDynamics;
                h = obj.DynamicModel.Holonomic; 
            
                NH = 3*NF; % number of holonmic constraints
                grf_values = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Ground_Reaction_Force"));
            
                % Extract initial variables
                x = [q{1};dq{1}]; 

                grf = {}; 
                for j = 1:N
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
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1},grf{1}),zeros(NB,1),zeros(NB,1));
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x,ddq{1}),zeros(NH,1),zeros(NH,1));
            
                % obj = FrictionConstraint(obj,grf{1},'Pyramid');
                % % Inital cost
                J = 0; 
        
                % Loop to apply the rest of the equality and collocation
                % constraints
                for i = 1:N-1 
                    % Extract variables 
                    x_curr = [q{i}; dq{i}]; 
                    x_next = [q{i+1};dq{i+1}]; 
                    dx_curr = [dq{i};ddq{i}];
                    dx_next = [dq{i+1};ddq{i+1}]; 
            
                    % Apply constraints
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x_next,ddq{i+1},tau{i+1},grf{i+1}),zeros(NB,1),zeros(NB,1));
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x_next,ddq{i+1}),zeros(NH,1),zeros(NH,1)); 
         
                    % Trapezoidal collocation
                    % x_eval = x_curr + t{i} * dt/2 * (dx_curr + dx_next); 
                     x_eval = x_curr + (t{i}/(N-1))/2 * (dx_curr + dx_next); 
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation",x_next - x_eval,zeros(NB*2,1),zeros(NB*2,1)); 
                                        
                    % Time continuity 
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity", t{i+1} - t{i}, 0, 0); 
                    
                    % Foot clearance
                    obj = FootConstraints(obj, x_next);

                    obj = FrictionConstraint(obj,grf{i+1},'Pyramid');

                    % Add cost 
                    % J = J + (Cost(obj.Cost.Type,tau{i},x_curr)+Cost(obj.Cost.Type,tau{i+1},x_next))*dt/2;
                    J = J + (Cost(obj.Cost.Type,tau{i},x_curr)+Cost(obj.Cost.Type,tau{i+1},x_next))*(t{i}/(N-1))/2;
                end
        
                % obj.Cost.Symbolic = t{1}*J;
                obj.Cost.Symbolic = J;
            end
        case 'Hermite-Simpson'
            % S = Variable('S', 1); S.Type = "Slack"; 
            % S = AddBounds(S,0, inf);
            
            NC = 2*N - 1; % number of collocation points
            % Check if phase is in contact
            if isempty(obj.ContactConfiguration)
                obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TAU], NC); 
            else
                NF = numel(obj.ContactConfiguration); % number of contact forces
                TotalGRFVariables = []; 
        
                for i = 1:NF
                    % GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-2500;-2500;0], [2500; 2500; 2500], [50;50;100]);
                    GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-1000;-1000;0], [1000;1000;1000]);
                    % GRF = Variable(['GRF_' num2str(obj.ContactConfiguration{i})], [3 1], [-1000;-1000;0]);
                    GRF.Name = ['GRF_' num2str(obj.ContactConfiguration{i})]; 
                    GRF.Type = "Ground_Reaction_Force"; 
                    GRF = AddBounds(GRF,varlow{idxGRF}, varup{idxGRF});
                    TotalGRFVariables = [TotalGRFVariables;GRF];
                end
        
                obj.VariableList = obj.VariableList.BuildVariableList([T;Q;DQ;DDQ;TAU;TotalGRFVariables], NC);
                 
            end
            
            t = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Time"));
            q = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Position"));
            dq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Velocity"));
            ddq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Acceleration"));
            tau = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Torque_Input"));
            % slack = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Slack"));
            
            % Configure Constraints
            if isempty(obj.ContactConfiguration)
                NF = 0; 
                f = obj.DynamicModel.Dynamics;
        
                % Extract variables
                x = [q{1};dq{1}]; 

                % Apply initial constraints
                if ~isempty(obj.InitialConfiguration)
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Initial Configuration", x - obj.InitialConfiguration, zeros(2*NB,1), zeros(2*NB,1));
                end
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1}),zeros(NB,1),zeros(NB,1));
                % obj = FootConstraints(obj, x);

                % Inital cost
                J = 0;

                % Loop to apply the rest of the equality and collocation
                % constraints
                for i = 1:N-1 
                    % Indicies
                    idx1 = 2*i-1; idx2 = 2*i; idx3 = 2*i+1; 

                    % Extract variables 
                    x1 = [q{idx1};dq{idx1}]; x2 = [q{idx2};dq{idx2}]; x3 = [q{idx3};dq{idx3}];
                    dx1 = t{idx1} * [dq{idx1};ddq{idx1}]; dx2 = t{idx2} * [dq{idx2};ddq{idx2}]; dx3 = t{idx3} * [dq{idx3};ddq{idx3}];
                    % dx1 = [dq{idx1};ddq{idx1}]; dx2 = [dq{idx2};ddq{idx2}]; dx3 = [dq{idx3};ddq{idx3}];
            
                    % % Equality constraint
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x2,ddq{idx2},tau{idx2}),zeros(NB,1),zeros(NB,1));
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x3,ddq{idx3},tau{idx3}),zeros(NB,1),zeros(NB,1));
              
                    % % Hermite Simpson collocation
                    x_mid = 1/2*(x1 + x3) + dt/8 * (dx1 - dx3);
                    % x_mid = 1/2*(x1 + x3) + (t{idx2}/(N-1))/8 * (dx1 - dx3);
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation Mid-Point",x2 - x_mid,zeros(NB*2,1),zeros(NB*2,1));
                    x_next = x1 + dt/6 * (dx1 + 4*dx2 + dx3);
                    % x_next = x1 + (t{idx3}/(N-1))/6 * (dx1 + 4*dx2 + dx3);
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation Next",x3 - x_next,zeros(NB*2,1),zeros(NB*2,1)); 

                    % Time continuity
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity",t{idx2} - t{idx1} ,0, 0);
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity",t{idx3} - t{idx2} ,0, 0);
               
                   % Foot clearance 
                   obj = FootConstraints(obj, x2);
                   obj = FootConstraints(obj, x3);

                   % Cost
                   % J = J + dt/6* (slack{idx1}+ 4*slack{idx2}+slack{idx3});
                   % P_1 = Cost(obj.Cost.Type,tau{idx1},x1);  P_2 = Cost(obj.Cost.Type,tau{idx2},x2);  P_3 = Cost(obj.Cost.Type,tau{idx3},x3); 
                    J = J + dt/6* (Cost(obj.Cost.Type,tau{idx1},x1) + 4*Cost(obj.Cost.Type,tau{idx2},x2) + Cost(obj.Cost.Type,tau{idx3},x3));

                   % J = J + (t{idx1}/(N-1))/6* (Cost(obj.Cost.Type,tau{idx1},x1) + 4*Cost(obj.Cost.Type,tau{idx2},x2) + Cost(obj.Cost.Type,tau{idx3},x3));
                    
                   % SlackVariables = [slack{idx1} - P_1; slack{idx2} - P_2; slack{idx3} - P_3 ];
                   % obj.ConstraintList = obj.ConstraintList.AddConstraint("Slack Variables",SlackVariables ,[0;0;0], [inf;inf;inf]);
            
                end
            
                obj.Cost.Symbolic = t{1} * J;
                % obj.Cost.Symbolic = J;
                % obj.Cost.Symbolic = t{1} * sumsqr(vertcat(tau{:}));

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
                for j = 1:NC
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
        
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x,ddq{1},tau{1},grf{1}),zeros(NB,1),zeros(NB,1));
                obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x,ddq{1}),zeros(NH,1),zeros(NH,1));
                % obj = FootConstraints(obj, x);
                % obj = FrictionConstraint(obj,grf{1},'Pyramid'); 
                % Inital cost
                J = 0;

                % Loop to apply the rest of the equality and collocation
                % constraints
                for i = 1:N-1 
                    % indeces
                    idx1 = 2*i-1; idx2 = 2*i; idx3 = 2*i+1; 
                    % Extract variables 
                    x1 = [q{idx1};dq{idx1}]; x2 = [q{idx2};dq{idx2}]; x3 = [q{idx3};dq{idx3}];
                    dx1 = t{idx1} * [dq{idx1};ddq{idx1}]; dx2 = t{idx2} * [dq{idx2};ddq{idx2}]; dx3 = t{idx3} * [dq{idx3};ddq{idx3}];
                    % dx1 = [dq{idx1};ddq{idx1}]; dx2 = [dq{idx2};ddq{idx2}]; dx3 = [dq{idx3};ddq{idx3}];

                    % Equality constraint
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x2,ddq{idx2},tau{idx2},grf{idx2}),zeros(NB,1),zeros(NB,1));
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Dynamics",f(x3,ddq{idx3},tau{idx3}, grf{idx3}),zeros(NB,1),zeros(NB,1));
                  
                    % Holonomic constraints
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x2,ddq{idx2}),zeros(NH,1),zeros(NH,1));
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Holonomic",h(x3,ddq{idx3}),zeros(NH,1),zeros(NH,1));
         
                    % Hermite Simpson collocation
                    x_mid = 1/2*(x1 + x3) + dt/8 * (dx1 - dx3);
                    % x_mid = 1/2*(x1 + x3) + (t{idx2}/(N-1))/8 * (dx1 - dx3);
                    
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation Mid-Point",x2 - x_mid,zeros(NB*2,1),zeros(NB*2,1));
                    x_next = x1 + dt/6 * (dx1 + 4*dx2 + dx3);
                    % x_next = x1 + (t{idx3}/(N-1))/6 * (dx1 + 4*dx2 + dx3);

                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Collocation Next",x3 - x_next,zeros(NB*2,1),zeros(NB*2,1)); 

                    % Time continuity
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity",t{idx2} - t{idx1} ,0, 0);
                    obj.ConstraintList = obj.ConstraintList.AddConstraint("Time Continuity",t{idx3} - t{idx2} ,0, 0);
     
                   % Foot clearance
                   obj = FootConstraints(obj, x2);
                   obj = FootConstraints(obj, x3);
                    
                   % obj = FrictionConstraint(obj,grf{idx2},'Pyramid'); 
                   % obj = FrictionConstraint(obj,grf{idx3},'Pyramid'); 

                   % Cost
                   % J = J + dt/6* (slack{idx1}+ 4*slack{idx2}+slack{idx3});
                   % P_1 = Cost(obj.Cost.Type,tau{idx1},x1);  P_2 = Cost(obj.Cost.Type,tau{idx2},x2);  P_3 = Cost(obj.Cost.Type,tau{idx3},x3); 
                   J = J + dt/6* (Cost(obj.Cost.Type,tau{idx1},x1) + 4*Cost(obj.Cost.Type,tau{idx2},x2) + Cost(obj.Cost.Type,tau{idx3},x3));
                    
                   % J = J + (t{idx1}/(N-1))/6* (Cost(obj.Cost.Type,tau{idx1},x1) + 4*Cost(obj.Cost.Type,tau{idx2},x2) + Cost(obj.Cost.Type,tau{idx3},x3));
                
                   % SlackVariables = [slack{idx1} - P_1; slack{idx2} - P_2; slack{idx3} - P_3 ];
                   % obj.ConstraintList = obj.ConstraintList.AddConstraint("Slack Variables",SlackVariables ,[0;0;0], [inf;inf;inf]);
            
                end
                obj.Cost.Symbolic = t{1} * J;
                % obj.Cost.Symbolic = J;
                % obj.Cost.Symbolic = J;
                % obj.Cost.Symbolic = t{1} * sumsqr(vertcat(tau{:}));
            end
        case 'Radau 2'
            obj = Collocation(obj, 2, 'radau'); 
            NF = numel(obj.ContactConfiguration);

        case 'Radau 3'
            obj = Collocation(obj, 3, 'radau'); 
            NF = numel(obj.ContactConfiguration);

        case 'Legendre 2'
            obj = Collocation(obj, 2, 'legendre'); 
            NF = numel(obj.ContactConfiguration);

        case 'Legendre 3'
            obj = Collocation(obj, 3, 'legendre'); 
            NF = numel(obj.ContactConfiguration);            

        otherwise
            disp('Collocation scheme not supported');
    end

    % Function to handle the transitions 
    % Extract final state

    q_final = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Position"));
    dq_final = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Velocity"));
    % x_final = [obj.VariableList.Symbolics{end-NF-3}; obj.VariableList.Symbolics{end-NF-2}]; 
    x_final = [q_final{end};dq_final{end}];
    if obj.Transition.Type == "Impact"
        obj.DynamicModel = obj.DynamicModel.SpecifyImpact(obj.Transition.Contacts); 
        I = obj.DynamicModel.Impact; 
        Next_state = I(x_final); 
        obj.Transition.NextState = Next_state; 
        obj = FootConstraintsImpact(obj,x_final); 
    elseif obj.Transition.Type == "Lift Off"
        grf_final = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Ground_Reaction_Force"));
        grf_final = grf_final(end-numel(obj.ContactConfiguration)+1:end);
        for j = 1:numel(obj.Transition.Contacts)
            index = find(strcmp(string(obj.Transition.Contacts{j}), string(obj.ContactConfiguration)));
            % gr_liftoff = obj.VariableList.Symbolics{end-NF+index}; 
            gr_liftoff = grf_final{index}; 
            obj.ConstraintList = obj.ConstraintList.AddConstraint("Lift Off " + obj.Transition.Contacts{j},gr_liftoff(3), 0, 0); 
        end
        obj.Transition.NextState = x_final;    
    else
        disp("Transition not supported"); 

    end

end