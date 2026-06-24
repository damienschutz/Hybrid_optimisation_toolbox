%% Random seeding sensititivity analysis
% clear; 
clc; 
tic; 

data_space = logspace(-1,2,50);
zeta = 0.5; 

%% Cheetah Parameters
mass_cheetah = 45.5; 
k_space_cheetah = mass_cheetah*data_space; 

%% Horse Parameters
mass_horse = 545.58; 
k_space_horse = mass_horse*data_space; 

X0 = CreateNewSeed(Data.PhaseData);
horse_transverse_12ms(X0,k_space_horse(38), zeta, counter);
    horse_transverse_12ms(X0,k_space_horse(49), zeta, counter);

    cheetah_transverse_12ms(X0,k_space_cheetah(2), zeta, counter); 
    cheetah_transverse_12ms(X0,k_space_cheetah(38), zeta, counter); 
    cheetah_transverse_12ms(X0,k_space_cheetah(49), zeta, counter); 
    

%% Transverse gallops
counter = 2; 
for i = 1:4
    X0 = 100*rand(5382,1);
    X0(2) = -0.1;

    X = horse_transverse_12ms(X0,k_space_horse(2), zeta, counter);
    X0 = X; 
    horse_transverse_12ms(X0,k_space_horse(38), zeta, counter);
    horse_transverse_12ms(X0,k_space_horse(49), zeta, counter);

    cheetah_transverse_12ms(X0,k_space_cheetah(2), zeta, counter); 
    cheetah_transverse_12ms(X0,k_space_cheetah(38), zeta, counter); 
    cheetah_transverse_12ms(X0,k_space_cheetah(49), zeta, counter); 
    
    counter = counter + 1; 
end
toc; 
disp("Transverse Optimisations finished")

%% Rotary gallops
tic;
counter = 1; 
for i = 1:5
    X0 = 100*rand(5328,1);
    X0(2) = -0.1;

    X = horse_rotary_12ms(X0,k_space_horse(2), zeta, counter);
    X0 = X; 
    horse_rotary_12ms(X0,k_space_horse(38), zeta, counter);
    horse_rotary_12ms(X0,k_space_horse(49), zeta, counter);

    cheetah_rotary_12ms(X0,k_space_cheetah(2), zeta, counter); 
    cheetah_rotary_12ms(X0,k_space_cheetah(38), zeta, counter); 
    cheetah_rotary_12ms(X0,k_space_cheetah(49), zeta, counter); 
    
    counter = counter + 1; 
end
toc; 
disp("Rotary Optimisations finished")

%%

showmotion(Cheetah,Data.Animation.T,Data.Animation.Q)