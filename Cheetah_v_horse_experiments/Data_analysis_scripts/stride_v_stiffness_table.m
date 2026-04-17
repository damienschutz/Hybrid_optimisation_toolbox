% =========================================================
% build_stride_stiffness_table.m
%
% Reads the four extracted CSVs and outputs a slimmed-down
% table of stride length vs spinal stiffness for every
% species x gait x speed combination.
%
% OUTPUT: stride_vs_stiffness.csv
% =========================================================

clear; clc;

% ---- INPUT CSV FILES ------------------------------------
CSV_FILES = {
    'cheetah_rotary_results.csv';
    'cheetah_transverse_results.csv';
    'horse_rotary_results.csv';
    'horse_transverse_results.csv';
};
OUT_CSV = 'stride_vs_stiffness.csv';
% ---------------------------------------------------------

% -- Load and concatenate all four CSVs -------------------
all_data = [];
for ci = 1:numel(CSV_FILES)
    fname = CSV_FILES{ci};
    if ~isfile(fname)
        warning('CSV not found, skipping: %s', fname);
        continue
    end
    T = readtable(fname, 'TextType', 'string');
    all_data = [all_data; T]; %#ok<AGROW>
end

if isempty(all_data)
    error('No data loaded. Check that CSV files exist in the current folder.');
end

fprintf('Loaded %d total rows.\n', height(all_data));

% -- Filter to successful solves only ---------------------
n_before = height(all_data);
all_data = all_data(all_data.solver_success == 1, :);
fprintf('Rows after filtering to solver_success==1: %d  (dropped %d)\n\n', ...
    height(all_data), n_before - height(all_data));

% -- Sort by species, gait, speed, then stiffness ---------
all_data = sortrows(all_data, ...
    {'species', 'gait_type', 'target_speed_m_s', 'stiffness_normalised'}, ...
    {'ascend',  'ascend',    'ascend',            'ascend'});

% -- Write slim CSV ---------------------------------------
header = {'species', 'gait_type', 'target_speed_ms', ...
          'stiffness_Nm_rad', 'stiffness_normalised', 'stride_length_m'};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));

for ri = 1:height(all_data)
    row = all_data(ri, :);
    fprintf(fid, '%s,%s,%g,%g,%g,%g\n', ...
        row.species, ...
        row.gait_type, ...
        row.target_speed_m_s, ...
        row.("stiffness_Nm_rad"), ...
        row.stiffness_normalised, ...
        row.stride_length_m);
end

fclose(fid);
fprintf('Done. Output written to: %s\n', OUT_CSV);