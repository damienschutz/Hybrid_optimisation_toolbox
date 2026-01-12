classdef OptimalControlProblem
    properties
        Name
        DynamicModel
        Phases
        Cost
        ConstraintList
        VariableList
        Parameter
        Problem
        Options
        Solution
        Data
        Seed
    end
    methods
        function obj = OptimalControlProblem(name)
            obj.Name = string(name); 
            
            % opts.ipopt.linear_solver = 'ma97'; 

            % N_threads = 8;
            % setenv("OMP_NUM_THREADS",num2str(N_threads));
            opts.ipopt.print_level = 0; 
            opts.ipopt.linear_scaling_on_demand = 'no'; 
            opts.ipopt.tol = 1e-3;
            % opts.ipopt.max_iter = 1000; 
            opts.ipopt.max_cpu_time = 5000; 
            opts.ipopt.nlp_scaling_method = 'gradient-based';
            opts.ipopt.hessian_approximation = 'limited-memory'; 
            opts.ipopt.mu_strategy = 'adaptive';
            opts.ipopt.recalc_y = 'yes';
            opts.ipopt.recalc_y_feas_tol = 1e-3;
            opts.ipopt.limited_memory_update_type = 'bfgs';
            opts.ipopt.limited_memory_max_history = 10; 
            % opts.detect_simple_bounds = true;
            

            obj.Options = opts; 

        end

        function obj = AddPhase(obj, phaselist)
            for i = 1:numel(phaselist)
                obj.Phases = [obj.Phases, phaselist(i)];
            end
        end

        function obj = BuildProblem(obj)
            X = []; LBX = []; UBX = []; 
            G = []; LBG = []; UBG = []; 
            J = 0; X0 = []; 
            
            obj.ConstraintList = ConstraintList();
            obj.VariableList = VariableList();
            

            for i = 1:numel(obj.Phases)
                currPhase = obj.Phases(i);

                obj.ConstraintList.Name = [obj.ConstraintList.Name, currPhase.ConstraintList.Name];
                obj.ConstraintList.List = [obj.ConstraintList.List, currPhase.ConstraintList.List];
                
                obj.VariableList.Symbolics = [obj.VariableList.Symbolics, currPhase.VariableList.Symbolics];
                obj.VariableList.Type = [obj.VariableList.Type, currPhase.VariableList.Type];

                X = [X; vertcat(currPhase.VariableList.Symbolics{:})];
                X0 = [X0;currPhase.VariableList.Init]; 
                LBX = [LBX; currPhase.VariableList.LowerBound];
                UBX = [UBX; currPhase.VariableList.UpperBound];

                G = [G; vertcat(currPhase.ConstraintList.List{:})];
                LBG = [LBG; currPhase.ConstraintList.LowerBound];
                UBG = [UBG; currPhase.ConstraintList.UpperBound];

                J = J + currPhase.Cost.Symbolic; 
            end

            obj.ConstraintList.LowerBound = LBG; 
            obj.ConstraintList.UpperBound = UBG; 

            obj.VariableList.LowerBound = LBX;
            obj.VariableList.UpperBound = UBX;

        
            X0(2) = -0.1;  
            obj.Problem.Values.X = X;
            obj.Problem.Values.X0 = X0;
            obj.Problem.Values.LBX = LBX;
            obj.Problem.Values.UBX = UBX;

            obj.Problem.Values.G = G;
            obj.Problem.Values.LBG = LBG;
            obj.Problem.Values.UBG = UBG;

            
            if isempty(obj.Cost)
                obj.Problem.J = J;

            elseif obj.Cost == "Cost of Transport"
                x_i = obj.VariableList.ExtractState("Initial");
                x_f = obj.VariableList.ExtractState("Final");
                m = obj.DynamicModel.Mass; 
                J_COT = J/( m * 9.81 * (x_f(1) - x_i(1)) );
                obj.Problem.J = J_COT;
                Jfunc = Function('J',{obj.Problem.Values.X}, {J_COT});
                obj.Problem.CostFunction = Jfunc; 
            
            elseif obj.Cost == "Smooth Cost of Transport"
                x_i = obj.VariableList.ExtractState("Initial");
                x_f = obj.VariableList.ExtractState("Final");
                m = obj.DynamicModel.Mass; 
                NB = obj.DynamicModel.Spatial_Robot.NB; 

                % J_COT = J/( m * 9.81 * (x_f(1) - x_i(1)) );
                J_COT = J/( m * 9.81 * x_f(1) );
  

                Jfunc_COT = Function('J',{obj.Problem.Values.X}, {J_COT});
                obj.Problem.CostFunction.COT = Jfunc_COT; 

                ddq = obj.VariableList.Symbolics(find(string(obj.VariableList.Type) == "Acceleration"));
                J_acc = sumsqr(vertcat(ddq{:})); 
                avgvel = obj.Parameter.Value;

                J_acc  = (1/(m*100))*J_acc/(NB * avgvel ); 
                Jfunc_acc = Function('J',{obj.Problem.Values.X}, {J_acc});
                obj.Problem.CostFunction.Acceleration = Jfunc_acc; 

                obj.Problem.J = J_COT + J_acc;
            else
                obj.Problem.J = J;
            end
            
            J = obj.Problem.J;

            if isempty(obj.Parameter)
                obj.Problem.NLP = struct('f',J,'x',X,'g',G);
            else
                p = obj.Parameter.Symbol; 
                obj.Problem.NLP = struct('f',J,'x',X,'g',G,'p',p);
            end
            
            import casadi.*
            obj.Problem.Solver = nlpsol('solver','ipopt',obj.Problem.NLP ,obj.Options); 
            
        end

        function obj = SolveNLP(obj, X_init)
            if nargin > 1
                obj.Problem.Values.X0 = X_init;
            else
            end

            X0 = obj.Problem.Values.X0; 
            LBX = obj.Problem.Values.LBX;
            UBX = obj.Problem.Values.UBX;

            LBG = obj.Problem.Values.LBG;
            UBG = obj.Problem.Values.UBG;

            solver = obj.Problem.Solver;

            if isempty(obj.Parameter)
                % tic;
                sol = solver('x0', X0, 'lbx', LBX, 'ubx', UBX, ...
                'lbg', LBG, 'ubg', UBG);
                % toc;

                obj.Solution = sol; 
                 
            else
                p = obj.Parameter.Value; 
                % tic;
                sol = solver('x0', X0, 'lbx', LBX, 'ubx', UBX, ...
                'lbg', LBG, 'ubg', UBG, 'p',p);
                % toc;

                obj.Solution = sol; 
                
            end
        end

        function obj = ConstraintJacobian(obj)
            import casadi.*
            spy(jacobian(obj.Problem.Values.G, obj.Problem.Values.X)); 
        end

        function obj = ExtractData(obj)
            import casadi.*
            X = full(obj.Solution.x); 

            obj.Data.Return = obj.Problem.Solver.stats(); 

            [phase_config, N] = CreatePhaseConfig(obj); 

            if size(obj.DynamicModel.Spatial_Robot.Xtree{1}) == [6,6]
                phase_data = ExtractMultiphaseSolution(X , phase_config);
            else
                phase_data = ExtractMultiphaseSolution2D(X , phase_config);
            end
            
            obj.Data.PhaseData = phase_data; 
            
            if obj.Cost == "Smooth Cost of Transport"
                obj.Data.Cost.Total = full(obj.Solution.f);
                accfunc = obj.Problem.CostFunction.Acceleration; cotfunc = obj.Problem.CostFunction.COT;
                obj.Data.Cost.Acceleration = full(accfunc(obj.Solution.x)); 
                obj.Data.Cost.COT = full(cotfunc(obj.Solution.x)); 
            else
                obj.Data.Cost = full(obj.Solution.f); 
            end

            % Generate animation Data
            q = []; 
            t = []; 
            start = 0; 
            for i = 1:numel(obj.Phases)
                q_curr = phase_data(i).position;
                t_curr = phase_data(i).time(1);
                q_curr = q_curr';
                q = [q, q_curr];
                t_span = linspace(start,start + t_curr,N);
                start = start + t_curr;
                t = [t, t_span];
            end

            obj.Data.Animation = struct('Q',q,'T', t);

        end

        function [phase_config, N] = CreatePhaseConfig(obj)
            n_dof = obj.DynamicModel.Spatial_Robot.NB; 
            n_phases = numel(obj.Phases); 
            phase_config = struct(); 
            intphase = obj.Phases(1); 
            N = intphase.Collocation.Points;

            if intphase.Collocation.Scheme == "Hermite-Simpson"
                N = 2*N - 1; 
            else
            end

            for i = 1:n_phases
                currphase = obj.Phases(i); 
                phase_config(i).n_dof = n_dof; 
                phase_config(i).n_points = N;
                phase_config(i).n_contacts = numel(currphase.ContactConfiguration); 
            end

        end

        function obj = Animate(obj)
            model = obj.DynamicModel.Spatial_Robot; 
            q = obj.Data.Animation.Q; 
            t = obj.Data.Animation.T; 

            showmotion(model,t,q)
        end

        function obj = NewSeed(obj, refinement)
            currN = obj.Phases(1).Collocation.Points; 
            refined_sol = RefineMultiphaseSolution(obj.Data.PhaseData, refinement); 
            new_seed = CreateNewSeed(refined_sol); 

            obj.Seed.Data = new_seed; 

            newN = (currN-1)*refinement+1; 
            obj.Seed.NewN = newN; 

        end
    end
end
