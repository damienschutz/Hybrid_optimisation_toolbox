%% Master script to collect all data in a single csv

clear; clc;

files(1) = dir('cheetah_rotary_results.csv');
files(2) = dir('cheetah_transverse_results.csv');
files(3) = dir('horse_rotary_results.csv');
files(4) = dir('horse_transverse_results.csv');

allT = table();

for k = 1:numel(files)
    T = readtable(fullfile(files(k).folder, files(k).name), ...
                  'VariableNamingRule', 'preserve');   % <-- add this
    allT = [allT; T];
end

writetable(allT, 'master_results.csv');

disp('Finished')