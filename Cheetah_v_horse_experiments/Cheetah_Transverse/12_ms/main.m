%% First inital run of the first test - random seeding
% clear; 
clc; 
tic; 

%%
mass = 45.5; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
zeta = 0.5; 
counter = 1; 
X0 = CreateNewSeed(Data.PhaseData); 
%%
for i = 1:numel(k_space)
    X = cheetah_transverse_12ms(X0, k_space(i), zeta, counter); 
    X0 = X; 
    counter = counter + 1; 
end
toc; 
disp("Optimisation finished")
toc;
%%
% toc;
showmotion(Cheetah,Data.Animation.T,Data.Animation.Q)
%