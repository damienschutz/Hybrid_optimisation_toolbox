%% First inital run of the first test - random seeding
clear; 
clc; 
tic; 

%%
mass = 45.5; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
zeta = 0.5; 
counter = 1; 

load("49.9836_k_cheetah_extended_12ms.mat");
X0 = CreateNewSeed(Data.PhaseData); 

%% For a fuul sweep
% for i = 1:numel(k_space)
%     X = cheetah_rotary_12ms(X0,k_space(i), zeta, counter); 
%     X0 = X; 
%     counter = counter + 1; 
% end
toc; 

tic;
% For a single result
X = cheetah_rotary_12ms(X0,k_space(18), zeta, counter); 
disp("Optimisation finished")
toc;
%%
showmotion(Cheetah,Data.Animation.T,Data.Animation.Q)