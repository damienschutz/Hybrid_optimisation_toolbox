function add_path_Hybrid_Optimisation_Toolbox()
% Function to add all the necessary scripts and folders
% to the MatLab path
% Must be run before any other scripts are run
% Created by: Damien Schutz 
% Date: 18/08/2025

cur = fullfile(pwd);
addpath(fullfile(cur));

% Code to run on Windows platform
addpath(genpath('Casadi'));
addpath(genpath('HSL_Solvers'));

% Required functions
addpath(genpath('Optim_functions'));

% Add spatial vector algebra suite
addpath(genpath('Spatial_casadi'));

% Add path to models
addpath(genpath('Models'));

% make sure the machine is set up with valid C compiler
% mex -setup C

end