%% Direct collocation tests for simple 2d hopper
clear;
clc; 
load("hop_in_place_2phase_seed.mat");

Hopper = DynamicModel(hopper); 
Hopper.Mass = 1;
N = 10; 

m = 1; l = 1; g = 1; mf = 0.05*m; ml = 0.1*m; mt = 0.85*m;  
kalpha = 5*m*g*l; kl = 10*m*g/l;                    % spring coefficients
zeta_l = 0.2*sqrt(2)*m*sqrt(g/l); zeta_alpha = 0.2*mf*g*l;  % damping ratios
bl = 1; balpha = 0.4;  % damping

Hopper = Hopper.AddSpringDamper(kalpha,balpha,4); 
Hopper = Hopper.AddSpringDamper(kl,bl,5); 

%% Joint Limits
poslimits = [10;2;0;pi/2;0.25]; neglimits = [-1;0.5;0;-pi/2;-0.75]; torquelimits = [0;20;20]; 
%% Phase 1 - Stance
P1 = Phase("Impact on ground", {}, N); 
P1.DynamicModel = Hopper; 
P1.Cost.Type = "Sum Torques"; 
P1 = P1.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {0.2,neglimits,-torquelimits}, {3,poslimits,torquelimits});
% P1 = P1.AddBoundsToPhase({"Time"}, {0.2}, {3});
P1 = P1.ConfigurePhase({"Foot"},"Lift Off",{"Foot"},"Hermite-Simpson"); 
P1 = BuildPhase(P1);

x1_init = P1.VariableList.ExtractState("Initial");

P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(1), 0, 0);
P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(3), 0, 0);
% P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(2), 1.5, 1.5);
P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(7), -1.005, -1.005);

%% Phase 2 - Lift off
P2 = Phase("Lift on ground", {}, N); 
P2.DynamicModel = Hopper; 
P2.Cost.Type = "Sum Torques"; 
P2 = P2.AddBoundsToPhase({"Time", "Position", "Torque_Input"}, {0.2,neglimits,-torquelimits}, {3,poslimits,torquelimits});
% P2 = P2.AddBoundsToPhase({"Time"}, {0.2}, {3});
P2 = P2.ConfigurePhase({},"Impact",{"Foot"},"Hermite-Simpson", P1.Transition.NextState); 
P2 = BuildPhase(P2);

x2_final = P2.Transition.NextState;

P2.ConstraintList = P2.ConstraintList.AddConstraint("Periodicity", x2_final - x1_init, zeros(10,1), zeros(10,1));
%% Build probelm
problem = OptimalControlProblem('Hop in place'); 
problem.DynamicModel = Hopper;
problem.Cost = "Sum Torques"; 
problem.Phases = [P1, P2];
problem = BuildProblem(problem);

%% Solve problem
solvedproblem = SolveNLP(problem);
%% Solve problem with seed
% solvedproblem = SolveNLP(problem, X0);

%% Animate solution
solvedproblem = solvedproblem.ExtractData(); 
% solvedproblem.Animate; 

%% Plots
plotting_2phase(solvedproblem);
%% Create new seed
refined_sol = RefineMultiphaseSolution2D(solvedproblem.Data.PhaseData,2); 
X0 = CreateNewSeed(solvedproblem.Data.PhaseData);
% X0 = CreateNewSeed(refined_sol);
% save('hop_forward_2phase_seed.mat',"X0");