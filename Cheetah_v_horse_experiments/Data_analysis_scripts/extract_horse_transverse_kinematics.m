% =========================================================
% extract_horse_transverse_kinematics.m
%
% Extracts stride kinematics for the horse transverse gait at
% representative normalised stiffness values and compares
% against literature values.
%
% Extracted per run:
%   - stride frequency (1 / stride period)
%   - stance time of front-right limb  (sum of phases 7 & 8)
%   - swing time  = stride period - stance time
%   - duty factor = stance time / stride period
%   - stride length
%
% OUTPUT: horse_transverse_kinematics.csv
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR    = '..';
FOLDER_NAME = 'Horse_Transverse';
MASS        = 545.18;
SPEEDS_MS   = [8, 10, 12, 15];

TARGET_NORM_STIFFNESSES = [0.1, 1, 10, 50, 100];
STIFFNESS_TOL           = 0.5;

% Front-right limb is in contact during these two phases
FR_STANCE_PHASES = [7, 8];

DOF_FORWARD = 1;

OUT_CSV = 'horse_transverse_kinematics.csv';

% =========================================================
% ---- CSV HEADER -----------------------------------------
% =========================================================

header = { ...
    'species', 'gait', 'target_speed_ms', ...
    'stiffness_Nm_rad', 'stiffness_normalised', 'stiffness_norm_requested', ...
    'stride_period_s', 'stride_frequency_Hz', 'stride_length_m', ...
    'stance_time_FR_s', 'swing_time_FR_s', 'duty_factor_FR', ...
    'notes' ...
};

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));

total = 0;

% =========================================================
% ---- MAIN LOOP ------------------------------------------
% =========================================================

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

    % Build normalised stiffness index
    avail_norm = nan(1, numel(mat_files));
    for si = 1:numel(mat_files)
        t = regexp(mat_files(si).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(t)
            avail_norm(si) = str2double(t{1}) / MASS;
        end
    end

    % ----------------------------------------------------------
    for ti = 1:numel(TARGET_NORM_STIFFNESSES)

        target_norm = TARGET_NORM_STIFFNESSES(ti);

        % Initialise row defaults
        notes          = '';
        stiffness_abs  = NaN;
        stiffness_norm = NaN;
        stride_period  = NaN;
        stride_freq    = NaN;
        stride_length  = NaN;
        stance_time    = NaN;
        swing_time     = NaN;
        duty_factor    = NaN;

        [min_dist, best_idx] = min(abs(avail_norm - target_norm));

        if min_dist > STIFFNESS_TOL
            notes = sprintf('closest=%.4f dist=%.4f from target=%.4f', ...
                avail_norm(best_idx), min_dist, target_norm);
            warning('%s / %d ms / target k=%.4f: %s', ...
                FOLDER_NAME, spd, target_norm, notes);
        end

        fpath = fullfile(speed_folder, mat_files(best_idx).name);
        fname = mat_files(best_idx).name;

        % Parse stiffness from filename
        tok = regexp(fname, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(tok)
            stiffness_abs  = str2double(tok{1});
            stiffness_norm = avail_norm(best_idx);
        else
            notes = append_note(notes, 'could not parse stiffness from filename');
        end

        % Load file
        load_ok = true;
        try
            S = load(fpath);
            if ~isfield(S, 'Data')
                notes   = append_note(notes, 'no Data struct');
                load_ok = false;
            else
                D = S.Data;
            end
        catch ME
            notes   = append_note(notes, ['load error: ' ME.message]);
            load_ok = false;
        end

        if ~load_ok
            % Write a flagged row and move on
            safe_source = lit_source;
            if contains(safe_source, ','); safe_source = ['"' safe_source '"']; end
            if contains(notes,       ','); notes       = ['"' notes       '"']; end
            fprintf(fid, ...
                'horse,transverse,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
                spd, stiffness_abs, stiffness_norm, target_norm, ...
                stride_period, stride_freq, stride_length, ...
                stance_time, swing_time, duty_factor, ...
                notes);
            total = total + 1;
            continue
        end

        % ---- Stride period & frequency ------------------
        try
            T             = D.Animation.T;
            stride_period = T(end) - T(1);
            stride_freq   = 1 / stride_period;
        catch
            notes = append_note(notes, 'Animation.T missing');
        end

        % ---- Stride length ------------------------------
        try
            Q             = D.Animation.Q;
            stride_length = Q(DOF_FORWARD, end) - Q(DOF_FORWARD, 1);
        catch
            notes = append_note(notes, 'Animation.Q error');
        end

        % ---- Front-right stance time --------------------
        % Sum durations of all FR stance phases
        try
            PD          = D.PhaseData;
            n_available = numel(PD);
            stance_time = 0;
            for ph = FR_STANCE_PHASES
                if ph <= n_available
                    t_ph        = PD(ph).time;
                    stance_time = stance_time + t_ph(1);
                else
                    notes = append_note(notes, ...
                        sprintf('FR phase %d exceeds n_phases=%d', ph, n_available));
                end
            end
        catch
            notes = append_note(notes, 'PhaseData error');
        end

        % ---- Swing time & duty factor -------------------
        if ~isnan(stride_period) && ~isnan(stance_time)
            swing_time  = stride_period - stance_time;
            duty_factor = stance_time   / stride_period;
        else
            notes = append_note(notes, 'swing/DF could not be computed');
        end

        fprintf(fid, ...
            'horse,transverse,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
            spd, stiffness_abs, stiffness_norm, target_norm, ...
            stride_period, stride_freq, stride_length, ...
            stance_time, swing_time, duty_factor, ...
            notes);

        total = total + 1;

        fprintf('  %d m/s | k_norm=%.4f (target %.4f) | T=%.3fs f=%.2fHz | stance=%.4fs | SL=%.3fm\n', ...
            spd, stiffness_norm, target_norm, ...
            stride_period, stride_freq, stance_time, stride_length);

    end % stiffness loop
end % speed loop

fclose(fid);
fprintf('\nDone. %d rows written to: %s\n', total, OUT_CSV);

% =========================================================
% ---- HELPER FUNCTION ------------------------------------
% =========================================================

function note = append_note(existing, new_msg)
    if isempty(existing)
        note = new_msg;
    else
        note = [existing '; ' new_msg];
    end
end