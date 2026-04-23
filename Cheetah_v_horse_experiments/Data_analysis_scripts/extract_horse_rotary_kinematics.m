% =========================================================
% extract_horse_rotary_kinematics.m
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
% OUTPUT: horse_rotary_kinematics.csv
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR    = '..';
FOLDER_NAME = 'Horse_Rotary';
MASS        = 545.18;
SPEEDS_MS   = [8, 10, 12, 15];

TARGET_NORM_STIFFNESSES = [0.1, 1, 10, 50, 100];
STIFFNESS_TOL           = 0.5;

% Front-right limb is in contact during these two phases
FR_STANCE_PHASES = [7, 8];
FL_STANCE_PHASES = [6, 7];

% Hind-left limb is in contact during these two phases
HL_STANCE_PHASES = [3, 4];
HR_STANCE_PHASES = [2, 3];

DOF_FORWARD = 1;

OUT_CSV = 'horse_rotary_kinematics.csv';

% =========================================================
% ---- CSV HEADER -----------------------------------------
% =========================================================

header = { ...
    'species', 'gait', 'target_speed_ms', ...
    'stiffness_Nm_rad', 'stiffness_normalised', 'stiffness_norm_requested', ...
    'stride_period_s', 'stride_frequency_Hz', 'stride_length_m', ...
    'stance_time_F_s', 'swing_time_F_s', 'duty_factor_F', ...
    'stance_time_H_s', 'swing_time_H_s', 'duty_factor_H', ...
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
        stance_time_h  = NaN;
        swing_time_h   = NaN;
        duty_factor_h  = NaN;

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
                'horse,rotary,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
                spd, stiffness_abs, stiffness_norm, target_norm, ...
                stride_period, stride_freq, stride_length, ...
                stance_time, swing_time, duty_factor, ...
                stance_time_h, swing_time_h, duty_factor_h, ...
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
            stance_time_fr = 0;
            for ph = FR_STANCE_PHASES
                if ph <= n_available
                    t_ph        = PD(ph).time;
                    stance_time_fr = stance_time_fr + t_ph(1);
                else
                    notes = append_note(notes, ...
                        sprintf('FR phase %d exceeds n_phases=%d', ph, n_available));
                end
            end
        catch
            notes = append_note(notes, 'PhaseData error');
        end

        % ---- Swing time & duty factor -------------------
        if ~isnan(stride_period) && ~isnan(stance_time_fr)
            swing_time_fr  = stride_period - stance_time_fr;
            duty_factor_fr = stance_time_fr   / stride_period;
        else
            notes = append_note(notes, 'swing/DF could not be computed');
        end

        % ---- Front-left stance time --------------------
        % Sum durations of all FR stance phases
        try
            PD          = D.PhaseData;
            n_available = numel(PD);
            stance_time_fl = 0;
            for ph = FL_STANCE_PHASES
                if ph <= n_available
                    t_ph        = PD(ph).time;
                    stance_time_fl = stance_time_fl + t_ph(1);
                else
                    notes = append_note(notes, ...
                        sprintf('FR phase %d exceeds n_phases=%d', ph, n_available));
                end
            end
        catch
            notes = append_note(notes, 'PhaseData error');
        end

        % ---- Swing time & duty factor -------------------
        if ~isnan(stride_period) && ~isnan(stance_time_fl)
            swing_time_fl  = stride_period - stance_time_fl;
            duty_factor_fl = stance_time_fl   / stride_period;
        else
            notes = append_note(notes, 'swing/DF could not be computed');
        end

        % ---- Total forelimb ---------------------------
        stance_time = (stance_time_fr+stance_time_fl)/2;
        swing_time  = (swing_time_fl+swing_time_fr)/2; 
        duty_factor = (duty_factor_fl+duty_factor_fr)/2;

        % ---- Hind-left stance time --------------------
        % Sum durations of all HL stance phases
        try
            PD          = D.PhaseData;
            n_available = numel(PD);
            stance_time_hl = 0;
            for ph = HL_STANCE_PHASES
                if ph <= n_available
                    t_ph        = PD(ph).time;
                    stance_time_hl = stance_time_hl + t_ph(1);
                else
                    notes = append_note(notes, ...
                        sprintf('HL phase %d exceeds n_phases=%d', ph, n_available));
                end
            end
        catch
            notes = append_note(notes, 'PhaseData error');
        end

        % ---- Swing time & duty factor -------------------
        if ~isnan(stride_period) && ~isnan(stance_time_hl)
            swing_time_hl  = stride_period - stance_time_hl;
            duty_factor_hl = stance_time_hl  / stride_period;
        else
            notes = append_note(notes, 'swing/DF could not be computed');
        end

        % ---- Hind-right stance time --------------------
        % Sum durations of all HL stance phases
        try
            PD          = D.PhaseData;
            n_available = numel(PD);
            stance_time_hr = 0;
            for ph = HR_STANCE_PHASES
                if ph <= n_available
                    t_ph        = PD(ph).time;
                    stance_time_hr = stance_time_hr + t_ph(1);
                else
                    notes = append_note(notes, ...
                        sprintf('HL phase %d exceeds n_phases=%d', ph, n_available));
                end
            end
        catch
            notes = append_note(notes, 'PhaseData error');
        end

        % ---- Swing time & duty factor -------------------
        if ~isnan(stride_period) && ~isnan(stance_time_hr)
            swing_time_hr  = stride_period - stance_time_hr;
            duty_factor_hr = stance_time_hr  / stride_period;
        else
            notes = append_note(notes, 'swing/DF could not be computed');
        end

        % ---- Total hindlimb ---------------------------
        stance_time_h = (stance_time_hl+stance_time_hr)/2;
        swing_time_h  = (swing_time_hl+swing_time_hr)/2; 
        duty_factor_h = (duty_factor_hl+duty_factor_hr)/2;


        fprintf(fid, ...
            'horse,rotary,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%s\n', ...
            spd, stiffness_abs, stiffness_norm, target_norm, ...
            stride_period, stride_freq, stride_length, ...
            stance_time, swing_time, duty_factor, ...
            stance_time_h, swing_time_h, duty_factor_h, ...
            notes);

        total = total + 1;

        % fprintf('  %d m/s | k_norm=%.4f (target %.4f) | T=%.3fs f=%.2fHz | stance=%.4fs | SL=%.3fm\n', ...
        %     spd, stiffness_norm, target_norm, ...
        %     stride_period, stride_freq, stance_time, stride_length);

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