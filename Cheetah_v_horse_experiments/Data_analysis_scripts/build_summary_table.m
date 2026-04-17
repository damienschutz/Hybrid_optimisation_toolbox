% =========================================================
% build_summary_table.m
%
% Reads the four extracted CSVs and produces a summary table
% with one row per species x gait x speed combination.
%
% NOTE: MATLAB's readtable sanitises column names on load,
% replacing special characters like / with _.
% So 'target_speed_m/s' becomes 'target_speed_m_s', etc.
% =========================================================

clear; clc;

% ---- INPUT CSV FILES ------------------------------------
CSV_FILES = {
    'cheetah_rotary_results.csv';
    'cheetah_transverse_results.csv';
    'horse_rotary_results.csv';
    'horse_transverse_results.csv';
};
OUT_CSV = 'summary_table.csv';
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

% -- Show actual column names so you can debug if needed --
fprintf('Loaded %d rows. Column names as seen by MATLAB:\n', height(all_data));
disp(all_data.Properties.VariableNames');

% -- Keep only successful solves --------------------------
n_before = height(all_data);
all_data = all_data(all_data.solver_success == 1, :);
fprintf('Rows after filtering to solver_success==1: %d  (dropped %d)\n\n', ...
    height(all_data), n_before - height(all_data));

% -- Write summary CSV ------------------------------------
sum_header = { ...
    'species', 'gait_type', 'target_speed_ms', ...
    'min_cost_of_transport', 'stiffness_norm_at_min', ...
    'cost_at_lowest_stiffness', 'cost_at_highest_stiffness', ...
    'pct_change_low_to_min', 'pct_change_min_to_high', ...
    'n_successful_runs' ...
};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(sum_header, ','));

% -- Get unique combinations ------------------------------
species_list = unique(all_data.species);

for si = 1:numel(species_list)
    sp = species_list(si);
    gait_list = unique(all_data.gait_type(all_data.species == sp));

    for gi = 1:numel(gait_list)
        gt = gait_list(gi);

        % Use the sanitised column name: / -> _
        speed_col  = all_data.target_speed_m_s;
        stiff_col  = all_data.stiffness_normalised;
        cost_col   = all_data.cost_COT;

        base_mask  = all_data.species == sp & all_data.gait_type == gt;
        speed_list = unique(speed_col(base_mask));

        for vi = 1:numel(speed_list)
            spd  = speed_list(vi);
            mask = base_mask & speed_col == spd;
            sub  = all_data(mask, :);

            if isempty(sub)
                continue
            end

            % Sort by normalised stiffness ascending
            [~, ord]    = sort(stiff_col(mask));
            sub         = sub(ord, :);
            costs       = sub.cost_COT;
            stiffnesses = sub.stiffness_normalised;
            n_runs      = height(sub);

            % Minimum cost and stiffness at minimum
            [min_cost, min_idx] = min(costs);
            stiff_at_min        = stiffnesses(min_idx);

            % Endpoint values
            cost_low  = costs(1);     % lowest stiffness tested
            cost_high = costs(end);   % highest stiffness tested

            % Percent changes (negative = improvement, positive = worsening)
            pct_low_to_min  = 100 * (min_cost - cost_low)  / cost_low;
            pct_min_to_high = 100 * (cost_high - min_cost) / min_cost;

            fprintf(fid, '%s,%s,%g,%g,%g,%g,%g,%.2f,%.2f,%d\n', ...
                sp, gt, spd, ...
                min_cost, stiff_at_min, ...
                cost_low, cost_high, ...
                pct_low_to_min, pct_min_to_high, ...
                n_runs);

            fprintf('  %s x %s x %g m/s  |  min=%.4f at k=%.3f  |  low=%.4f  high=%.4f  |  %%low>min=%.1f  %%min>high=%.1f\n', ...
                sp, gt, spd, min_cost, stiff_at_min, ...
                cost_low, cost_high, pct_low_to_min, pct_min_to_high);
        end
    end
end

fclose(fid);
fprintf('\nSummary written to: %s\n', OUT_CSV);