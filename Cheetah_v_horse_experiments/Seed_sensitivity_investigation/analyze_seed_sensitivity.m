% =========================================================
% analyze_seed_sensitivity.m
%
% Analyses multi-seed sensitivity runs stored in:
%   sensitivity_analysis/
%       {stiffness}_k_{label}_12ms{seed}.mat
%       e.g.  500_k_cheetah_extended_12ms3.mat
%
% For each (label x stiffness) group of 5 seeds, reports:
%   - success rate  (Data.Return.return_status == 'Solve_Succeeded')
%   - best objective value   (min Data.Cost.Total among successes)
%   - worst objective value  (max Data.Cost.Total among successes)
%   - spread = max - min
%   - standard deviation
%   - reference objective from the original single-seed dataset
%     (closest matching normalised stiffness in original folder)
%   - delta = best_seed - reference
%   - delta_pct = 100 * (best_seed - reference) / reference
%
% Filename-to-original-folder mapping (edit if needed):
%   cheetah_extended  ->  Cheetah_Rotary/12_ms/
%   cheetah_grounded  ->  Cheetah_Transverse/12_ms/
%   horse_extended    ->  Horse_Rotary/12_ms/
%   horse_grounded    ->  Horse_Transverse/12_ms/
%
% OUTPUT: seed_sensitivity_summary.csv
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

SENS_DIR  = 'Optimisation_data';   % folder containing the new .mat files
ORIG_BASE = '..';                       % root for original dataset folders

% Mapping: sensitivity label -> { original_folder, mass_kg, display_name }
LABEL_MAP = struct( ...
    'cheetah_extended', {{'Cheetah_Rotary',    45.5,  'Cheetah Rotary'}}, ...
    'cheetah_grounded', {{'Cheetah_Transverse',45.5,  'Cheetah Transverse'}}, ...
    'horse_extended',   {{'Horse_Rotary',       545.58, 'Horse Rotary'}}, ...
    'horse_grounded',   {{'Horse_Transverse',   545.58, 'Horse Transverse'}} ...
);

SUCCESS_STATUS = 'Solve_Succeeded';   % string to match in Data.Return.return_status
STIFF_TOL      = 0.5;                % tolerance for matching normalised stiffness
                                      % to original dataset

OUT_CSV = 'seed_sensitivity_summary.csv';

% =========================================================
% ---- SCAN SENSITIVITY FOLDER ------------------------------
% ========================================================= 

if ~isfolder(SENS_DIR)
    error('Sensitivity folder not found: %s', SENS_DIR);
end

mat_files = dir(fullfile(SENS_DIR, '*.mat'));
if isempty(mat_files)
    error('No .mat files found in: %s', SENS_DIR);
end

fprintf('Found %d .mat files in %s\n', numel(mat_files), SENS_DIR);

% ---- Parse filename fields --------------------------------
% Expected pattern: {stiffness}_k_{label}_12ms{seed}.mat
% e.g.  500_k_cheetah_extended_12ms3.mat

pattern = '^(-?[\d.]+)_k_([a-zA-Z_]+)_12ms(\d+)\.mat$';

records = struct('stiffness_abs', {}, 'label', {}, 'seed', {}, ...
                 'fpath', {}, 'success', {}, 'cost', {});

n_parsed   = 0;
n_skipped  = 0;

for fi = 1:numel(mat_files)
    fname = mat_files(fi).name;
    tok   = regexp(fname, pattern, 'tokens', 'once');

    if isempty(tok)
        warning('  Could not parse filename: %s — skipping.', fname);
        n_skipped = n_skipped + 1;
        continue
    end

    stiff_abs = str2double(tok{1});
    label     = tok{2};
    seed      = str2double(tok{3});
    fpath     = fullfile(SENS_DIR, fname);

    % Load file
    try
        S = load(fpath);
        if ~isfield(S, 'Data')
            warning('  No Data struct in: %s', fname);
            n_skipped = n_skipped + 1;
            continue
        end
        D = S.Data;
    catch ME
        warning('  Load error in %s: %s', fname, ME.message);
        n_skipped = n_skipped + 1;
        continue
    end

    % Extract success status
    try
        ret_status = strtrim(D.Return.return_status);
        is_success = strcmpi(ret_status, SUCCESS_STATUS);
    catch
        is_success = false;
        warning('  return_status missing in: %s', fname);
    end

    % Extract cost
    try
        cost = D.Cost.Total;
    catch
        cost = NaN;
        warning('  Cost.Total missing in: %s', fname);
    end

    n_parsed = n_parsed + 1;
    records(n_parsed).stiffness_abs = stiff_abs;
    records(n_parsed).label         = label;
    records(n_parsed).seed          = seed;
    records(n_parsed).fpath         = fpath;
    records(n_parsed).success       = is_success;
    records(n_parsed).cost          = cost;
end

fprintf('Parsed: %d files   Skipped: %d\n\n', n_parsed, n_skipped);

% =========================================================
% ---- LOAD ORIGINAL REFERENCE DATA -------------------------
% For each known label, load all costs from the original folder
% so we can match by normalised stiffness later.
% =========================================================

orig_data = struct();   % orig_data.(label).k_norm, .costs

known_labels = fieldnames(LABEL_MAP);
for li = 1:numel(known_labels)
    lbl   = known_labels{li};
    info  = LABEL_MAP.(lbl);
    orig_folder = info{1};
    mass        = info{2};

    orig_path = fullfile(ORIG_BASE, orig_folder, '12_ms');

    if ~isfolder(orig_path)
        warning('Original folder not found for %s: %s', lbl, orig_path);
        orig_data.(lbl).k_norm = [];
        orig_data.(lbl).costs  = [];
        continue
    end

    orig_files = dir(fullfile(orig_path, '*_k_*.mat'));
    n_orig     = numel(orig_files);
    k_raw      = nan(1, n_orig);
    c_raw      = nan(1, n_orig);

    for fi = 1:n_orig
        tok2 = regexp(orig_files(fi).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if isempty(tok2); continue; end
        k_raw(fi) = str2double(tok2{1}) / mass;
        try
            S2 = load(fullfile(orig_path, orig_files(fi).name));
            if isfield(S2, 'Data') && isfield(S2.Data, 'Cost')
                c_raw(fi) = S2.Data.Cost.Total;
            end
        catch; end
    end

    valid = ~isnan(k_raw) & ~isnan(c_raw);
    orig_data.(lbl).k_norm = k_raw(valid);
    orig_data.(lbl).costs  = c_raw(valid);
    fprintf('Loaded %d original reference points for %s\n', sum(valid), lbl);
end

% =========================================================
% ---- GROUP BY (label x stiffness) AND SUMMARISE -----------
% =========================================================

% Get unique (label, stiffness_abs) combinations
all_labels = {records.label};
all_k_abs  = [records.stiffness_abs];

groups = unique([all_k_abs', string(all_labels')], 'rows');   % as strings
% Re-parse to numeric
unique_pairs = {};
for gi = 1:size(groups, 1)
    unique_pairs{end+1} = {str2double(groups(gi,1)), char(groups(gi,2))}; %#ok<AGROW>
end

% Sort by label then stiffness
unique_pairs = vertcat(unique_pairs{:});
unique_pairs = sortrows(unique_pairs);

% ---- Write CSV --------------------------------------------
header = { ...
    'display_name', 'label', 'stiffness_abs', 'stiffness_norm', ...
    'n_seeds', 'n_success', 'success_rate_pct', ...
    'best_cost', 'worst_cost', 'spread_max_min', 'std_cost', ...
    'ref_cost', 'ref_stiffness_norm', 'ref_stiff_match_dist', ...
    'delta_best_minus_ref', 'delta_pct' ...
};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));

for gi = 1:size(unique_pairs,1)
    k_abs = unique_pairs{gi,1};
    lbl   = unique_pairs{gi,2};

    % Get info for this label
    if isfield(LABEL_MAP, lbl)
        info = LABEL_MAP.(lbl);
        mass = info{2};
        disp_name = info{3};
    else
        mass = NaN;
        disp_name = lbl;
        warning('Unknown label: %s — mass unknown, stiffness_norm will be NaN.', lbl);
    end

    k_norm = k_abs / mass;

    % Collect seeds for this group
    mask = [records.stiffness_abs] == k_abs & strcmp({records.label}, lbl);
    grp  = records(mask);

    n_seeds   = numel(grp);
    success_v = [grp.success];
    cost_v    = [grp.cost];

    n_success    = sum(success_v);
    success_rate = 100 * n_success / n_seeds;

    % Only use successful runs for cost stats
    succ_costs = cost_v(success_v & ~isnan(cost_v));

    if isempty(succ_costs)
        best_cost  = NaN;
        worst_cost = NaN;
        spread     = NaN;
        std_cost   = NaN;
    else
        best_cost  = min(succ_costs);
        worst_cost = max(succ_costs);
        spread     = worst_cost - best_cost;
        std_cost   = std(succ_costs);
    end

    % Match to original reference
    ref_cost       = NaN;
    ref_k_norm     = NaN;
    ref_match_dist = NaN;
    delta          = NaN;
    delta_pct      = NaN;

    if isfield(orig_data, lbl) && ~isempty(orig_data.(lbl).k_norm)
        orig_k = orig_data.(lbl).k_norm;
        orig_c = orig_data.(lbl).costs;
        [dist, best_ref_idx] = min(abs(orig_k - k_norm));
        ref_k_norm     = orig_k(best_ref_idx);
        ref_match_dist = dist;
        if dist <= STIFF_TOL
            ref_cost = orig_c(best_ref_idx);
            if ~isnan(best_cost) && ~isnan(ref_cost)
                delta     = best_cost - ref_cost;
                delta_pct = 100 * delta / ref_cost;
            end
        else
            warning('  %s k=%.4f: ref match dist %.4f > tol %.4f — ref_cost left as NaN.', ...
                lbl, k_norm, dist, STIFF_TOL);
        end
    end

    fprintf(fid, '%s,%s,%g,%g,%d,%d,%.1f,%g,%g,%g,%g,%g,%g,%g,%g,%.2f\n', ...
        disp_name, lbl, k_abs, k_norm, ...
        n_seeds, n_success, success_rate, ...
        best_cost, worst_cost, spread, std_cost, ...
        ref_cost, ref_k_norm, ref_match_dist, ...
        delta, delta_pct);

    fprintf('  %-25s  k_norm=%.4f  success=%d/%d  best=%.4f  spread=%.4f  ref=%.4f  delta=%.4f (%.1f%%)\n', ...
        disp_name, k_norm, n_success, n_seeds, ...
        best_cost, spread, ref_cost, delta, delta_pct);
end

fclose(fid);
fprintf('\nDone. Summary written to: %s\n', OUT_CSV);