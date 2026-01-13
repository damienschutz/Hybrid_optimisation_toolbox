%% Grounded transverse gallop horse - 8ms
function X = horse_transverse_12ms(X0, k, zeta, counter)
% clear;
% clc;
import casadi.*
%% Bounds

ex_front = 1.097; ex_hind = 0.87; % height 1.61
I_horse = 33.4713; b = zeta*2*sqrt(I_horse*k);

k_legs = 55000; b_legs = 750; 

phi = pi/4;
abmin = 5*pi/180; 
abmax = pi/8; 

LBPos = [-inf;-0.8;-inf;-phi;-phi;-phi;-phi;-abmin;-phi;0.93;-abmax;-phi;0.93;-abmin;-phi; 0.55;-abmax;-phi ;0.55];
UBPos = [inf; 0.5; inf; phi; phi; phi; phi; abmax ; phi; ex_front ;  abmin;phi; ex_front ; abmax;phi; ex_hind ;abmin ;phi ;ex_hind];

% Load model
Quadruped = DynamicModel(Horse);
Quadruped = Quadruped.AddSpringDamper(k,b,7); 

Quadruped = Quadruped.AddSpringDamperBounds(k_legs,b_legs,10, 1.02); 
Quadruped = Quadruped.AddSpringDamperBounds(k_legs,b_legs,13, 1.02); 
Quadruped = Quadruped.AddSpringDamperBounds(k_legs,b_legs,16, 0.71); 
Quadruped = Quadruped.AddSpringDamperBounds(k_legs,b_legs,19, 0.71); 

Quadruped.Mass = 545.18; 
collocation = "Hermite-Simpson";
N = 5;
%%

mintime = 0.02; 

vellimit = 200*ones(19,1); acclimit = inf*ones(19,1); 

prism = 550*10*3; 

torquelimits = [5000;1100;3300;prism;1100;3300;prism;3400;10200;prism;3400;10200;prism]; 

grflb = [-50*Quadruped.Mass;-50*Quadruped.Mass;0]; grfub = [50*Quadruped.Mass;50*Quadruped.Mass;50*Quadruped.Mass]; 

costfunc = "Sum Torques";

%% First Phase
P1 = Phase("Phase 1", {}, N);
P1.DynamicModel = Quadruped;
P1.Cost.Type = costfunc;
P1 = P1.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P1 = P1.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P1 = P1.ConfigurePhase({},"Impact", "HL", collocation); 
P1 = BuildPhase(P1); 

x1_init = P1.VariableList.ExtractState("Initial");

P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(1:2), [0;0], [0;0]);
P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start roll", x1_init(4), 0, 0);
P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start yaw", x1_init(6), 0, 0);
%%
P2 = Phase("Phase 2", {}, N);
P2.DynamicModel = Quadruped;
P2.Cost.Type = costfunc;
P2 = P2.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P2 = P2.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P2 = P2.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P2 = P2.ConfigurePhase("HL", "Impact", "HR", collocation, P1.Transition.NextState); 
P2 = BuildPhase(P2); 

%%
P3 = Phase("Phase 3", {}, N);
P3.DynamicModel = Quadruped;
P3.Cost.Type = costfunc;
P3 = P3.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P3 = P3.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P3 = P3.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P3 = P3.ConfigurePhase({"HL", "HR"}, "Lift Off", {"HL"}, collocation, P2.Transition.NextState); 
P3 = BuildPhase(P3); 
%%
P4 = Phase("Phase 4", {}, N);
P4.DynamicModel = Quadruped;
P4.Cost.Type = costfunc;
P4 = P4.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P4 = P4.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P4 = P4.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P4 = P4.ConfigurePhase({"HR"}, "Impact", {"FL"}, collocation, P3.Transition.NextState); 
P4 = BuildPhase(P4); 

%%
P5 = Phase("Phase 5", {}, N);
P5.DynamicModel = Quadruped;
P5.Cost.Type = costfunc;
P5 = P5.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P5 = P5.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P5 = P5.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P5 = P5.ConfigurePhase({"HR","FL"}, "Lift Off", {"HR"}, collocation, P4.Transition.NextState); 
P5 = BuildPhase(P5); 

%%
P6 = Phase("Phase 6", {}, N);
P6.DynamicModel = Quadruped;
P6.Cost.Type = costfunc;
P6 = P6.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P6 = P6.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P6 = P6.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P6 = P6.ConfigurePhase({"FL"}, "Impact", {"FR"},collocation,  P5.Transition.NextState); 
P6 = BuildPhase(P6); 

%%
P7 = Phase("Phase 7", {}, N);
P7.DynamicModel = Quadruped;
P7.Cost.Type = costfunc;
P7 = P7.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P7 = P7.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P7 = P7.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P7 = P7.ConfigurePhase({"FL", "FR"}, "Lift Off", {"FL"},collocation,  P6.Transition.NextState); 
P7 = BuildPhase(P7); 
%%
P8 = Phase("Phase 8", {}, N);
P8.DynamicModel = Quadruped;
P8.Cost.Type = costfunc;
P8 = P8.AddBoundsToPhase({"Velocity", "Acceleration"}, {-vellimit, -acclimit}, {vellimit, acclimit});
P8 = P8.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {mintime, LBPos, -torquelimits}, {1, UBPos, torquelimits});
P8 = P8.AddBoundsToPhase({"Ground_Reaction_Force"}, {grflb}, {grfub});
P8 = P8.ConfigurePhase({"FR"}, "Lift Off", {"FR"},collocation,  P7.Transition.NextState); 
P8 = BuildPhase(P8); 

x8_final = P8.VariableList.ExtractState("Final");

%%
p = MX.sym('p',1); 

t1 = P1.VariableList.ExtractTime; t2 = P2.VariableList.ExtractTime; t3 = P3.VariableList.ExtractTime; 
t4 = P4.VariableList.ExtractTime; t5 = P5.VariableList.ExtractTime; t6 = P6.VariableList.ExtractTime; 
t7 = P7.VariableList.ExtractTime; t8 = P8.VariableList.ExtractTime;

P8.ConstraintList = P8.ConstraintList.AddConstraint("Periodicity", x8_final(2:end) - x1_init(2:end), zeros(37,1), zeros(37,1));
P8.ConstraintList = P8.ConstraintList.AddConstraint("Average Speed", x8_final(1) - p*(t1+t2+t3+t4+t5+t6+t7+t8), 0, 0); 
P8.ConstraintList = P8.ConstraintList.AddConstraint("Total Time", (t1+t2+t3+t4+t5+t6+t7+t8), 0.25, 0.6); 

%% Build optimisation problem

problem = OptimalControlProblem('Transverse Gallop'); 
problem.DynamicModel = Quadruped; 
problem.Cost = "Smooth Cost of Transport";
problem.Phases = [P1,P2,P3,P4,P5,P6,P7,P8];
problem.Parameter.Symbol = p; 
problem.Parameter.Value = 12; 
problem = BuildProblem(problem);
%%
% problem.ConstraintJacobian();

%%
% X0 = load("horse_seed_grounded_N5.mat");
% X0 = X0.X_new;
solvedproblem = SolveNLP(problem, X0);

%% Create new seed and extract data
solvedproblem = solvedproblem.ExtractData(); 
% solvedproblem = solvedproblem.NewSeed(2);
% solvedproblem.Animate();

% new_seed = solvedproblem.Seed.Data; 
% N = solvedproblem.Seed.NewN; 

%% Save data
Data = solvedproblem.Data;
save(append(num2str(k),'_k_horse_grounded_12ms.mat'),"Data")
X = full(solvedproblem.Solution.x);
disp(append('Optimisation_no_',num2str(counter),'_complete'))
end

