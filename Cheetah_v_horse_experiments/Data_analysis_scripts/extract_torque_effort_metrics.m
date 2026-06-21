% =========================================================
% extract_torque_effort_metrics.m
%
% For every saved solution across all species, gaits, speeds
% and spinal stiffness values, computes torque-based effort
% metrics:
%
%   1. Sum(tau/tau_max)^2  — summed over all 13 actuated DOFs
%                            and all 72 collocation points
%   2. Peak |tau|/tau_max  — overall maximum normalised torque
%   3. 95th percentile |tau|/tau_max
%   4. % of stride time spent with ANY joint above 80% capacity
%   5. % of stride time spent with ANY joint above 90% capacity
%   6. Total positive joint work (J), if velocity (qdot) data
%      is available — integrated using trapz over real time
%
% Torque column layout (13 actuated DOFs):
%   col 1      : spine
%   cols 2-4   : limb 1 (front)  [ad/abduction, pitch, prismatic force]
%   cols 5-7   : limb 2 (front)  [ad/abduction, pitch, prismatic force]
%   cols 8-10  : limb 3 (hind)   [ad/abduction, pitch, prismatic force]
%   cols 11-13 : limb 4 (hind)   [ad/abduction, pitch, prismatic force]
%
% This ordering was inferred from data_analysis_v2.m, where
% position(:,10) (= torque col 4, front limb force) was
% labelled "front right spring" and position(:,19)
% (= torque col 13, hind limb force) was labelled
% "hind left spring". If this ordering is wrong for your
% model, edit TAU_MAX construction below.
%
% velocity is read from PhaseData(ph).velocity, using the
% same column offset as position (actuated DOF i -> column
% i+6, since the free-flyer body occupies the first 6 cols).
%
% OUTPUT: torque_effort_metrics.csv
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR = '..';

% { folder_name, species, gait, mass }
CASES = {
    'Cheetah_Rotary',     'cheetah', 'rotary',     45.5;
    'Cheetah_Transverse', 'cheetah', 'transverse', 45.5;
    'Horse_Rotary',       'horse',   'rotary',     545.18;
    'Horse_Transverse',   'horse',   'transverse', 545.18;
};

SPEEDS_MS = [8, 10, 12, 15];

OUT_CSV = 'torque_effort_metrics.csv';

% Thresholds for "time spent above X% torque capacity"
THRESH_1 = 0.80;
THRESH_2 = 0.90;

% =========================================================
% ---- ACTUATION LIMITS (Nm or N) --------------------------
% Tables 4.5 (cheetah) and 4.10 (horse)
% =========================================================

LIMITS.cheetah.spine             = 375;
LIMITS.cheetah.shoulder_adab     = 75;
LIMITS.cheetah.shoulder_pitch    = 375;
LIMITS.cheetah.hip_adab          = 75;
LIMITS.cheetah.hip_pitch         = 525;
LIMITS.cheetah.prismatic         = 1500;

LIMITS.horse.spine                = 5000;
LIMITS.horse.shoulder_adab        = 1100;
LIMITS.horse.shoulder_pitch       = 3300;
LIMITS.horse.hip_adab             = 3400;
LIMITS.horse.hip_pitch            = 10200;
LIMITS.horse.prismatic            = 16500;

% =========================================================
% ---- CSV HEADER -------------------------------------------
% =========================================================

header = { ...
    'species', 'gait', 'target_speed_ms', ...
    'stiffness_Nm_rad', 'stiffness_normalised', ...
    'sum_tau_norm_sq', 'peak_tau_norm', 'p95_tau_norm', ...
    sprintf('pct_stride_above_%d', round(THRESH_1*100)), ...
    sprintf('pct_stride_above_%d', round(THRESH_2*100)), ...
    'positive_joint_work_J', ...
    'solver_success', 'notes' ...
};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));

total_runs  = 0;
failed_runs = 0;

% =========================================================
% ---- MAIN LOOP --------------------------------------------
% =========================================================

for ci = 1:size(CASES, 1)

    folder_name = CASES{ci, 1};
    species     = CASES{ci, 2};
    gait        = CASES{ci, 3};
    mass        = CASES{ci, 4};

    % Build TAU_MAX vector for this species (13 actuated DOFs)
    L = LIMITS.(species);
    TAU_MAX = [ ...
        L.spine, ...                                  % col 1  spine
        L.shoulder_adab, L.shoulder_pitch, L.prismatic, ...  % cols 2-4  limb1 (front)
        L.shoulder_adab, L.shoulder_pitch, L.prismatic, ...  % cols 5-7  limb2 (front)
        L.hip_adab,      L.hip_pitch,      L.prismatic, ...  % cols 8-10 limb3 (hind)
        L.hip_adab,      L.hip_pitch,      L.prismatic  ...  % cols 11-13 limb4 (hind)
    ];

    for spd = SPEEDS_MS

        speed_folder = fullfile(BASE_DIR, folder_name, sprintf('%d_ms', spd));

        if ~isfolder(speed_folder)
            warning('Folder not found: %s  skipping.', speed_folder);
            continue
        end

        mat_files = dir(fullfile(speed_folder, '*_k_*.mat'));
        if isempty(mat_files)
            warning('No *_k_*.mat files in %s', speed_folder);
            continue
        end

        % -- Sort by stiffness ascending ----------------------
        k_vals = nan(1, numel(mat_files));
        for si = 1:numel(mat_files)
            t = regexp(mat_files(si).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
            if ~isempty(t); k_vals(si) = str2double(t{1}); end
        end
        [~, sort_idx] = sort(k_vals);
        mat_files = mat_files(sort_idx);

        fprintf('\nProcessing %s / %d m/s  (%d files)\n', folder_name, spd, numel(mat_files));

        for fi = 1:numel(mat_files)

            fpath = fullfile(speed_folder, mat_files(fi).name);
            fname = mat_files(fi).name;

            % -- Parse stiffness ------------------------------
            tok = regexp(fname, '^(-?[\d.]+)_k_', 'tokens', 'once');
            if ~isempty(tok)
                stiffness_abs  = str2double(tok{1});
                stiffness_norm = stiffness_abs / mass;
            else
                stiffness_abs  = NaN;
                stiffness_norm = NaN;
                warning('Could not parse stiffness from: %s', fname);
            end

            % -- Initialise row defaults -----------------------
            notes           = '';
            sum_tau_sq      = NaN;
            peak_tau_norm   = NaN;
            p95_tau_norm    = NaN;
            pct_above_1     = NaN;
            pct_above_2     = NaN;
            pos_work        = NaN;
            solver_success  = NaN;

            % -- Load file --------------------------------------
            load_ok = true;
            try
                S = load(fpath);
                if ~isfield(S, 'Data')
                    notes   = 'no Data struct';
                    load_ok = false;
                else
                    D = S.Data;
                end
            catch ME
                notes   = ['load error: ' ME.message];
                load_ok = false;
            end

            if ~load_ok
                if contains(notes, ','); notes = ['"' notes '"']; end
                fprintf(fid, '%s,%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
                    species, gait, spd, stiffness_abs, stiffness_norm, ...
                    sum_tau_sq, peak_tau_norm, p95_tau_norm, ...
                    pct_above_1, pct_above_2, pos_work, ...
                    solver_success, notes);
                total_runs  = total_runs + 1;
                failed_runs = failed_runs + 1;
                continue
            end

            % -- Solver success ----------------------------------
            try
                solver_success = double(D.Return.success);
            catch
                notes = [notes 'Return missing; '];
            end

            % -- Reconstruct torque, velocity and time traces ----
            try
                n_phases = numel(D.PhaseData);
                n_pts    = 9;
                n_total  = n_pts * n_phases;
                n_dof    = 13;

                tau_all  = zeros(n_total, n_dof);
                qdot_all = zeros(n_total, n_dof);
                time_vec = zeros(1, n_total);

                has_velocity = true;

                count    = 1;
                t_offset = 0;
                for ph = 1:n_phases
                    idx = count : count + n_pts - 1;

                    tau_all(idx, :) = D.PhaseData(ph).torque;   % [9 x 13]

                    if isfield(D.PhaseData(ph), 'velocity') && ...
                       size(D.PhaseData(ph).velocity, 2) >= 19
                        qdot_all(idx, :) = D.PhaseData(ph).velocity(:, 7:19);
                    else
                        has_velocity = false;
                    end

                    phase_dur = D.PhaseData(ph).time(1, 1);
                    time_vec(idx) = linspace(t_offset, t_offset + phase_dur, n_pts);

                    count    = count    + n_pts;
                    t_offset = t_offset + phase_dur;
                end

                % -- Normalised torque matrix [n_total x 13] -----
                tau_norm = abs(tau_all) ./ TAU_MAX;   % broadcasts TAU_MAX across rows

                % -- Metric 1: Sum(tau/tau_max)^2 -----------------
                sum_tau_sq = sum(tau_norm(:).^2);

                % -- Metric 2: Peak |tau|/tau_max -----------------
                peak_tau_norm = max(tau_norm(:));

                % -- Metric 3: 95th percentile --------------------
                p95_tau_norm = local_percentile(tau_norm(:), 95);

                % -- Metric 4/5: % stride above thresholds --------
                % At each time sample, take the max normalised
                % torque across all 13 joints, then determine the
                % time-weighted fraction above each threshold using
                % trapezoidal integration of the indicator function.
                max_per_sample = max(tau_norm, [], 2);   % [n_total x 1]

                ind_1 = double(max_per_sample > THRESH_1);
                ind_2 = double(max_per_sample > THRESH_2);

                stride_period = time_vec(end) - time_vec(1);
                if stride_period > 0
                    time_above_1 = trapz(time_vec, ind_1);
                    time_above_2 = trapz(time_vec, ind_2);
                    pct_above_1  = 100 * time_above_1 / stride_period;
                    pct_above_2  = 100 * time_above_2 / stride_period;
                else
                    notes = [notes 'invalid stride period; '];
                end

                % -- Metric 6: Positive joint work ----------------
                if has_velocity
                    power_all = tau_all .* qdot_all;        % [n_total x 13]
                    pos_power = max(power_all, 0);
                    work_per_joint = zeros(1, n_dof);
                    for j = 1:n_dof
                        work_per_joint(j) = trapz(time_vec, pos_power(:, j));
                    end
                    pos_work = sum(work_per_joint);
                else
                    notes = [notes 'velocity data unavailable; '];
                end

            catch ME
                notes = [notes 'metric calc error: ' ME.message '; '];
            end

            % -- Write row to CSV ---------------------------------
            if contains(notes, ','); notes = ['"' notes '"']; end
            fprintf(fid, '%s,%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
                species, gait, spd, stiffness_abs, stiffness_norm, ...
                sum_tau_sq, peak_tau_norm, p95_tau_norm, ...
                pct_above_1, pct_above_2, pos_work, ...
                solver_success, notes);
            total_runs = total_runs + 1;
            if ~isempty(strtrim(notes))
                failed_runs = failed_runs + 1;
            end

        end % file loop
    end % speed loop
end % case loop

fclose(fid);
fprintf('\n================================================\n');
fprintf('Done.  Total runs: %d   Flagged: %d\n', total_runs, failed_runs);
fprintf('Output: %s\n', OUT_CSV);

% =========================================================
% ---- LOCAL FUNCTIONS --------------------------------------
% (defined at end of script file — valid in MATLAB R2016b+)
% =========================================================

function p = local_percentile(data, pct)
% Linear-interpolation percentile, matching MATLAB's default
% prctile behaviour, without requiring the Statistics Toolbox.
    data = sort(data(:));
    n    = numel(data);
    if n == 0
        p = NaN;
        return
    end
    % MATLAB prctile uses (i - 0.5)/n as the percentile of the
    % i-th sorted value
    ranks = (((1:n) - 0.5) / n) * 100;
    if pct <= ranks(1)
        p = data(1);
    elseif pct >= ranks(end)
        p = data(end);
    else
        p = interp1(ranks, data, pct, 'linear');
    end
end