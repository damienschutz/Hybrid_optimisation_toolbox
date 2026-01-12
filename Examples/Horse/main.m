%% Generate rigid torso horse transverse gallop
clear; 
clc; 
tic; 
%%
mass = 545.18; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
zeta = 0.5; 
counter = 1; 
load(54518_k_horse_grounded_12ms".mat")
X0 = CreateNewSeed(Data.PhaseData);

X = horse_transverse_12ms(X0, k_space(50), zeta, counter); 
disp("Optimisation finished")

%%
% toc;
showmotion(Horse,Data.Animation.T,Data.Animation.Q)
