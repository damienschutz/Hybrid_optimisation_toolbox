%% Direct collocation tests for simple 2d hopper
clc;
clear;
load("hop_forward_2phase_seed.mat");
%%
import casadi.*
Hopper = DynamicModel(hopper); 
Hopper.Mass = 1;

kalpha = 5; kl = 50;                    % spring coefficients
bl = 2; balpha = 0.4; 

Hopper = Hopper.AddSpringDamper(kalpha,balpha,4); 
Hopper = Hopper.AddSpringDamper(kl,bl,5); 
%% Joint Limits
poslimits = [10;1.2;pi/4;pi/2;0.75]; neglimits = [-1;0.5;-pi/4;-pi/2;-0.5]; torquelimits = [0;20;20]; 
%% Phase 1
N = 10; 
P1 = Phase("Stance", {}, N); 
P1.DynamicModel = Hopper; 
P1.Cost.Type = "Sum Torques"; 
P1 = P1.AddBoundsToPhase({"Time", "Position","Torque_Input"}, {0.1,neglimits,-torquelimits}, {inf,poslimits,torquelimits});
% P1 = P1.AddBoundsToPhase({"Time"}, {0.1}, {inf});
P1 = P1.ConfigurePhase({"Foot"},"Lift Off",{"Foot"},"Hermite-Simpson"); 
P1 = BuildPhase(P1);

x1_init = P1.VariableList.ExtractState("Initial");

P1.ConstraintList = P1.ConstraintList.AddConstraint("Initial start", x1_init(1), 0, 0);

%% Phase 2
P2 = Phase("Lift off and impact", {}, N); 
P2.DynamicModel = Hopper; 
P2.Cost.Type = "Sum Torques"; 
P2 = P2.AddBoundsToPhase({"Time", "Position","Torque_Input"}, {0.1,neglimits,-torquelimits}, {inf,poslimits,torquelimits});
% P2 = P2.AddBoundsToPhase({"Time"}, {0.1}, {inf});
P2 = P2.ConfigurePhase({},"Impact",{"Foot"},"Hermite-Simpson", P1.Transition.NextState); 
P2 = BuildPhase(P2);

x2_final = P2.Transition.NextState;

%% Add periodicity
p = MX.sym('p',1); 

t1 = P1.VariableList.ExtractTime; t2 = P2.VariableList.ExtractTime;

P2.ConstraintList = P2.ConstraintList.AddConstraint("Periodicity", x2_final(2:end) - x1_init(2:end), zeros(9,1), zeros(9,1));
P2.ConstraintList = P2.ConstraintList.AddConstraint("Average Speed", x2_final(1) - p*(t1+t2), 0, 0); 
%% Build probelm
problem = OptimalControlProblem('Hop forward'); 
problem.DynamicModel = Hopper;
problem.Cost = "Cost of Transport"; 
problem.Phases = [P1, P2];
problem.Parameter.Symbol = p; 
problem.Parameter.Value = 1; 
problem = BuildProblem(problem);
%%
ConstraintJacobian(problem)
%% Solve problem
solvedproblem = SolveNLP(problem);
%% solve with seed
% solvedproblem = SolveNLP(problem, X0);

%% Animate solution
solvedproblem = solvedproblem.ExtractData(); 
%%
solvedproblem.Animate; 
%% Plots
% plotting_2phase(solvedproblem);
%% Create new seed
% X0 = CreateNewSeed(solvedproblem.Data.PhaseData);
% save('hop_forward_2phase_seed.mat',"X0");

%% Refine seed
% refined_sol = RefineMultiphaseSolution2D(solvedproblem.Data.PhaseData,2); 
% X0_ref = CreateNewSeed(refined_sol);
% save('hop_forward_2phase_refined_seed.mat',"X0_ref");