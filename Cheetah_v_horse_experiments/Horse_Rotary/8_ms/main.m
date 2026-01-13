%% First inital run of the first test - random seeding
% clear; 
clc; 
tic; 

%%
mass = 545.18; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
zeta = 0.5; 
counter = 1; 
X0 = CreateNewSeed(Data.PhaseData); 
%%
% for i = 1:numel(k_space)
%     X = horse_rotary_8ms(X0, k_space(i), zeta, counter); 
%     X0 = X; 
%     counter = counter + 1; 
% end
% toc; 
% disp("Optimisation finished")
% toc;

for i = 11:17
    X = horse_rotary_8ms(X0, k_space(i), zeta, counter); 
    X0 = X; 
    counter = counter + 1; 
end
toc; 
disp("Optimisation finished")
toc;

%%
showmotion(Horse,Data.  Animation.T,Data.Animation.Q)
%%
torque = []; 
for i = 1:8
    torque = [torque;Data.PhaseData(i).torque(:,4)];
end

hold on
plot(torque)
%%
vel = []; 
for i = 1:8
    vel = [vel;Data.PhaseData(i).velocity(:,1)];
end

hold on
plot(vel)
%%
% grf = []; 
% for i = 1:8
%     grf = [grf;Data.PhaseData(i).grf(:,3)];
% end

hold on
plot(Data.PhaseData(2).ground_reaction_forces(:,3))

%%
pos = []; 
for i = 1:8
    pos = [pos;Data.PhaseData(i).position(:,3)];
end

hold on
plot(pos)