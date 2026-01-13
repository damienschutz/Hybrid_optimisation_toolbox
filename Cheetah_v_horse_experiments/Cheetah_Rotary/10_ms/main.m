%% First inital run of the first test - random seeding
% clear; 
clc; 
tic; 
X0 = CreateNewSeed(Data.PhaseData);

%%
mass = 45.5; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
zeta = 0.5; 
counter = 1; 
%%
for i = 1:numel(k_space)
    X = cheetah_rotary_10ms(X0,k_space(i), zeta, counter); 
    X0 = X; 
    counter = counter + 1; 
end
toc; 
disp("Optimisation finished")
toc;
%%
showmotion(Cheetah,Data.Animation.T,Data.Animation.Q)
