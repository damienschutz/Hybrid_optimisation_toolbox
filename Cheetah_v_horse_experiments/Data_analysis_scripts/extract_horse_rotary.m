% =========================================================
% extract_horse_rotary.m
% OUTPUT: horse_rotary_results.csv
% =========================================================

clear; clc;

% ---- USER CONFIGURATION ---------------------------------
BASE_DIR    = '..';
SPECIES     = 'horse';
MASS        = 545.18;
GAIT        = 'rotary';
FOLDER_NAME = 'Horse_Rotary';
SPEEDS_MS   = [8, 10, 12, 15];
OUT_CSV     = 'horse_rotary_results.csv';

DOF_FORWARD = 1;   % row index in Animation.Q for forward translation
DOF_SPINE   = 7;   % row index in Animation.Q for spine/lumbar angle

% ---- DUTY FACTOR PHASE CONFIGURATION --------------------
% For each limb, list the PhaseData indices that correspond
% to its stance phase(s). Leave empty [] until you know
% which phases map to which limb for this gait.
% Example: if LF is in stance during phase 2, set LF = [2]
% If a limb spans two phases, list both: LF = [2, 6]
LIMB_PHASES.LF = [6, 7];   % Left  Fore
LIMB_PHASES.RF = [7, 8];   % Right Fore
LIMB_PHASES.LH = [3, 4];   % Left  Hind
LIMB_PHASES.RH = [2, 3];   % Right Hind
FLIGHT_PHASE   = [1, 5];
% ---------------------------------------------------------

header = { ...
    'species', 'mass_kg', 'gait_type', 'target_speed_m/s', ...
    'stiffness_Nm/rad', 'stiffness_normalised', ...
    'return_status', 'unified_return_status', 'solver_success', ...
    'iter_count', ...
    'cost_total', 'cost_acceleration', 'cost_COT', ...
    'stride_period_s', 'stride_length_m', 'achieved_avg_speed_m/s', ...
    'n_phases', 'n_stance_phases', 'total_stance_time_s', 'flight_time_s', ...
    'spine_peak_deg', 'spine_min_deg', 'spine_ROM_deg', ...
    'duty_factor_LF', 'duty_factor_RF', 'duty_factor_LH', 'duty_factor_RH', ...
    'notes' ...
};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));

total_runs  = 0;
failed_runs = 0;

for spd = SPEEDS_MS

    speed_folder = fullfile(BASE_DIR, FOLDER_NAME, sprintf('%d_ms', spd));

    if ~isfolder(speed_folder)
        warning('Folder not found: %s  skipping.', speed_folder);
        continue
    end

    mat_files = dir(fullfile(speed_folder, '*_k_*.mat'));

    if isempty(mat_files)
        warning('No *_k_*.mat files in %s', speed_folder);
        continue
    end

    % -- Sort files by stiffness value (ascending) ----------
    k_vals = nan(1, numel(mat_files));
    for si = 1:numel(mat_files)
        t = regexp(mat_files(si).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(t)
            k_vals(si) = str2double(t{1});
        end
    end
    [~, sort_idx] = sort(k_vals);
    mat_files = mat_files(sort_idx);
    % -------------------------------------------------------

    fprintf('\nProcessing %s / %d m/s  (%d files)\n', FOLDER_NAME, spd, numel(mat_files));

    for fi = 1:numel(mat_files)

        fpath = fullfile(speed_folder, mat_files(fi).name);
        fname = mat_files(fi).name;

        % -- Parse stiffness from filename ------------------
        tok = regexp(fname, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(tok)
            stiffness      = str2double(tok{1});
            stiffness_norm = stiffness / MASS;
        else
            stiffness      = NaN;
            stiffness_norm = NaN;
            warning('Could not parse stiffness from: %s', fname);
        end

        % -- Initialise all fields to safe defaults ---------
        notes          = '';
        return_status  = 'N/A';
        unified_status = 'N/A';
        success        = NaN;
        iter_count     = NaN;
        cost_total     = NaN;
        cost_accel     = NaN;
        cost_COT       = NaN;
        stride_period  = NaN;
        stride_length  = NaN;
        avg_speed      = NaN;
        n_phases       = NaN;
        n_stance       = NaN;
        spine_peak     = NaN;
        spine_min      = NaN;
        spine_ROM      = NaN;
        df_LF          = NaN;
        df_RF          = NaN;
        df_LH          = NaN;
        df_RH          = NaN;
        stance_time    = 0;
        flight_time    = 0;

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
            fprintf(fid, '%s,%g,%s,%g,%g,%g,%s,%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
                SPECIES, MASS, GAIT, spd, stiffness, stiffness_norm, ...
                return_status, unified_status, ...
                success, iter_count, ...
                cost_total, cost_accel, cost_COT, ...
                stride_period, stride_length, avg_speed, ...
                n_phases, n_stance, stance_time, flight_time, ...
                spine_peak, spine_min, spine_ROM, ...
                df_LF, df_RF, df_LH, df_RH, notes);
            total_runs  = total_runs + 1;
            failed_runs = failed_runs + 1;
            continue
        end

        % -- Return / solver info ---------------------------
        try
            return_status  = D.Return.return_status;
            unified_status = D.Return.unified_return_status;
            success        = double(D.Return.success);
            iter_count     = D.Return.iter_count;
        catch
            notes = [notes 'Return fields missing; '];
        end

        % -- Cost fields ------------------------------------
        try
            cost_total = D.Cost.Total;
            cost_accel = D.Cost.Acceleration;
            cost_COT   = D.Cost.COT;
        catch
            notes = [notes 'Cost fields missing; '];
        end

        % -- Stride period ----------------------------------
        try
            T             = D.Animation.T;
            stride_period = T(end) - T(1);
        catch
            notes = [notes 'Animation.T missing; '];
        end

        % -- Stride length & achieved speed -----------------
        try
            Q             = D.Animation.Q;
            dx            = Q(DOF_FORWARD, end) - Q(DOF_FORWARD, 1);
            stride_length = dx;
            if ~isnan(stride_period) && stride_period > 0
                avg_speed = dx / stride_period;
            end
        catch
            notes = [notes 'Animation.Q error; '];
        end

        % -- Phase count and stance count -------------------
        try
            PD       = D.PhaseData;
            n_phases = numel(PD);
            n_stance = 0;
            for ph = 1:n_phases
                grf = PD(ph).ground_reaction_forces;
                if ~isempty(grf) && any(grf(:) ~= 0)
                    n_stance = n_stance + 1;
                end
            end
        catch
            notes = [notes 'PhaseData error; '];
        end

        % -- Spine angle stats ------------------------------
        try
            spine_deg  = rad2deg(D.Animation.Q(DOF_SPINE, :));
            spine_peak = max(spine_deg);
            spine_min  = min(spine_deg);
            spine_ROM  = spine_peak - spine_min;
        catch
            notes = [notes 'Spine DOF error; '];
        end

        % -- Duty factors -----------------------------------
        % Phase duration = PD(ph).time(end) - PD(ph).time(1)
        % Duty factor    = total stance duration / stride period
        % Edit LIMB_PHASES at the top to assign phase indices.
        try
            if ~isnan(stride_period) && stride_period > 0
                limb_names  = {'LF', 'RF', 'LH', 'RH'};
                df_vals     = nan(1, 4);
                for li = 1:4
                    phase_ids = LIMB_PHASES.(limb_names{li});
                    if ~isempty(phase_ids)
                        stance_dur = 0;
                        for ph = phase_ids
                            stance_dur = stance_dur + ...
                                PD(ph).time(1);
                        end
                        df_vals(li) = stance_dur / stride_period;
                    end
                    % NaN left in df_vals if phase_ids is empty
                end
                df_LF = df_vals(1);
                df_RF = df_vals(2);
                df_LH = df_vals(3);
                df_RH = df_vals(4);
            else
                notes = [notes 'stride_period invalid for duty factor; '];
            end
        catch
            notes = [notes 'Duty factor error; '];
        end

        for fli = 1:numel(FLIGHT_PHASE)
            flid = FLIGHT_PHASE(fli);
            flight_time = flight_time + PD(flid).time(1);
        end

        stance_time = stride_period - flight_time; 

        % -- Wrap strings containing commas in quotes -------
        if contains(return_status,  ','); return_status  = ['"' return_status  '"']; end
        if contains(unified_status, ','); unified_status = ['"' unified_status '"']; end
        if contains(notes,          ','); notes          = ['"' notes          '"']; end

        % -- Write row to CSV --------------------------------
        fprintf(fid, '%s,%g,%s,%g,%g,%g,%s,%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
            SPECIES, MASS, GAIT, spd, stiffness, stiffness_norm, ...
            return_status, unified_status, ...
            success, iter_count, ...
            cost_total, cost_accel, cost_COT, ...
            stride_period, stride_length, avg_speed, ...
            n_phases, n_stance, stance_time, flight_time, ...
            spine_peak, spine_min, spine_ROM, ...
            df_LF, df_RF, df_LH, df_RH, notes);

        total_runs = total_runs + 1;
        if ~isempty(strtrim(notes))
            failed_runs = failed_runs + 1;
        end

    end % file loop
end % speed loop

fclose(fid);
fprintf('\n================================================\n');
fprintf('Done.  Total runs: %d   Flagged: %d\n', total_runs, failed_runs);
fprintf('Output: %s\n', OUT_CSV);