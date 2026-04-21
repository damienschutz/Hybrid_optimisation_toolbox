% =========================================================
% plot_spring_energy.m
%
% Plots potential energy stored in the spinal spring and two
% distal leg springs over a full stride, for a set of
% user-defined cases (species x gait x speed x stiffness).
%
% Each case produces one panel in a tiled layout.
% Raw energy + time data is also exported to a CSV file.
%
% Energy formula (from data_analysis_v2.m):
%   E_spine = (1/2) * k_spine * theta_spine^2
%   E_leg   = (1/2) * k_leg   * (theta_leg - theta_rest)^2
%
% The spine stiffness k here is the ABSOLUTE value (Nm/rad),
% read directly from the .mat filename.
%
% DOF indices:
%   DOF 7  = spine/lumbar angle   -> spine spring
%   DOF 10 = front-right leg DOF  -> FR leg spring
%   DOF 19 = hind-left  leg DOF   -> HL leg spring
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR = '..';

% ---- Species-specific parameters ------------------------
PARAMS.cheetah.spine_k        = 4550;
PARAMS.cheetah.leg_k          = 5000;
PARAMS.cheetah.leg_rest_front = 0.25;
PARAMS.cheetah.leg_rest_hind  = 0.25;
PARAMS.cheetah.mass           = 45.5;

PARAMS.horse.spine_k          = 54518;
PARAMS.horse.leg_k            = 55000;
PARAMS.horse.leg_rest_front   = 1.02;
PARAMS.horse.leg_rest_hind    = 0.71;
PARAMS.horse.mass             = 545.18;

% ---- Cases to plot --------------------------------------
% Each row: { folder_name, species, gait, speed_ms,
%             target_norm_stiffness, panel_title }
CASES = {
    'Cheetah_Rotary',   'cheetah', 'rotary',    12,  0.1,  'Rotary Cheetah (0.1\timesBW) at 12 m/s';
    'Cheetah_Rotary',   'cheetah', 'rotary',    12,  100,  'Rotary Cheetah (100\timesBW) at 12 m/s';
    'Horse_Transverse', 'horse',   'transverse',12,  0.1,  'Transverse Horse (0.1\timesBW) at 12 m/s';
    'Horse_Transverse', 'horse',   'transverse',12,  100,  'Transverse Horse (100\timesBW) at 12 m/s';
};

% ---- Layout ---------------------------------------------
N_COLS = 2;

% ---- Output files ---------------------------------------
OUT_FIG = 'spring_energy.fig';
OUT_PDF = 'spring_energy.pdf';
OUT_CSV = 'spring_energy_data.csv';

% ---- DOF indices ----------------------------------------
DOF_SPINE      = 7;
DOF_LEG_FRONT  = 10;
DOF_LEG_HIND   = 19;

% ---- Stiffness match tolerance (normalised units) -------
STIFFNESS_TOL = 0.5;

% =========================================================
% ---- FIGURE SETUP ---------------------------------------
% =========================================================

n_cases = size(CASES, 1);
n_rows  = ceil(n_cases / N_COLS);

fh = figure('Units', 'centimeters', 'Position', [2 2 18 12]);
set(fh, 'Color', 'w');
tl = tiledlayout(n_rows, N_COLS, ...
    'TileSpacing', 'compact', 'Padding', 'compact'); 

CLR_SPINE = [0.00, 0.45, 0.70];
CLR_FRONT = [0.85, 0.33, 0.10];
CLR_HIND  = [0.00, 0.62, 0.45];

% =========================================================
% ---- CSV SETUP ------------------------------------------
% =========================================================

fid = fopen(OUT_CSV, 'w');
fprintf(fid, '%s\n', strjoin({ ...
    'species', 'gait', 'speed_ms', ...
    'stiffness_Nm_rad', 'stiffness_normalised', 'stiffness_norm_requested', ...
    'time_s', ...
    'E_spine_J', 'E_front_leg_J', 'E_hind_leg_J'}, ','));

% =========================================================
% ---- MAIN LOOP ------------------------------------------
% =========================================================

for ci = 1:n_cases

    folder_name  = CASES{ci, 1};
    species      = CASES{ci, 2};
    gait         = CASES{ci, 3};
    speed_ms     = CASES{ci, 4};
    target_norm  = CASES{ci, 5};
    panel_title  = CASES{ci, 6};

    p    = PARAMS.(species);
    mass = p.mass;

    % ---- Find closest .mat file -------------------------
    speed_folder = fullfile(BASE_DIR, folder_name, sprintf('%d_ms', speed_ms));

    if ~isfolder(speed_folder)
        warning('Folder not found: %s — skipping panel %d.', speed_folder, ci);
        nexttile; title(sprintf('MISSING: %s', panel_title)); continue
    end

    mat_files = dir(fullfile(speed_folder, '*_k_*.mat'));
    if isempty(mat_files)
        warning('No .mat files in %s — skipping panel %d.', speed_folder, ci);
        nexttile; title(sprintf('NO FILES: %s', panel_title)); continue
    end

    % Build normalised stiffness index
    avail_norm = nan(1, numel(mat_files));
    for si = 1:numel(mat_files)
        t = regexp(mat_files(si).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(t)
            avail_norm(si) = str2double(t{1}) / mass;
        end
    end

    [min_dist, best_idx] = min(abs(avail_norm - target_norm));
    if min_dist > STIFFNESS_TOL
        warning('Panel %d: closest match %.4f is %.3f away from target %.4f.', ...
            ci, avail_norm(best_idx), min_dist, target_norm);
    end

    fpath = fullfile(speed_folder, mat_files(best_idx).name);

    % ---- Load .mat --------------------------------------
    try
        S = load(fpath);
        D = S.Data;
    catch ME
        warning('Could not load %s: %s', fpath, ME.message);
        nexttile; title(sprintf('LOAD ERROR: %s', panel_title)); continue
    end

    % ---- Reconstruct time axis and position traces ------
    n_phases = numel(D.PhaseData);
    n_pts    = 9;

    spine_pos = zeros(1, n_pts * n_phases);
    front_pos = zeros(1, n_pts * n_phases);
    hind_pos  = zeros(1, n_pts * n_phases);
    time_vec  = [];

    count    = 1;
    t_offset = 0;

    for ph = 1:n_phases
        idx = count : count + n_pts - 1;

        spine_pos(idx) = D.PhaseData(ph).position(:, DOF_SPINE)';
        front_pos(idx) = D.PhaseData(ph).position(:, DOF_LEG_FRONT)';
        hind_pos(idx)  = D.PhaseData(ph).position(:, DOF_LEG_HIND)';

        phase_dur = D.PhaseData(ph).time(1, 1);
        time_vec  = [time_vec, linspace(t_offset, t_offset + phase_dur, n_pts)]; 

        count    = count    + n_pts;
        t_offset = t_offset + phase_dur;
    end

    % ---- Compute spring potential energies --------------
    tok = regexp(mat_files(best_idx).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
    if ~isempty(tok)
        k_spine_abs = str2double(tok{1});
    else
        k_spine_abs = p.spine_k;
        warning('Panel %d: could not parse stiffness from filename, using default.', ci);
    end
    k_norm_actual = avail_norm(best_idx);

    E_spine = (0.5 * k_spine_abs) .* spine_pos.^2;
    E_front = (0.5 * p.leg_k)     .* (front_pos - p.leg_rest_front).^2;
    E_hind  = (0.5 * p.leg_k)     .* (hind_pos  - p.leg_rest_hind ).^2;

    % ---- Plot panel -------------------------------------
    ax = nexttile;
    hold(ax, 'on');

    plot(ax, time_vec, E_spine, '-',  'Color', CLR_SPINE, 'LineWidth', 1.4);
    plot(ax, time_vec, E_front, '--', 'Color', CLR_FRONT, 'LineWidth', 1.4);
    plot(ax, time_vec, E_hind,  ':',  'Color', CLR_HIND,  'LineWidth', 1.8);

    xlabel(ax, 'Time (s)',             'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, 'Potential energy (J)', 'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax,   panel_title,           'FontSize', 8, 'FontName', 'Times New Roman', ...
        'FontWeight', 'bold');

    set(ax, 'FontSize', 8, 'FontName', 'Times New Roman', ...
        'Box', 'on', 'TickDir', 'out');
    xlim(ax, [0, t_offset]);

    hold(ax, 'off');

    % ---- Write this case to CSV -------------------------
    % One row per time point
    for ri = 1:numel(time_vec)
        fprintf(fid, '%s,%s,%g,%g,%g,%g,%g,%g,%g,%g\n', ...
            species, gait, speed_ms, ...
            k_spine_abs, k_norm_actual, target_norm, ...
            time_vec(ri), ...
            E_spine(ri), E_front(ri), E_hind(ri));
    end

    fprintf('Panel %d done: %s  |  k_norm=%.4f (target %.4f)  |  k_abs=%.1f Nm/rad  |  %d rows written\n', ...
        ci, panel_title, k_norm_actual, target_norm, k_spine_abs, numel(time_vec));
end

% ---- Close CSV ------------------------------------------
fclose(fid);
fprintf('\nCSV written: %s\n', OUT_CSV);

% ---- Shared legend (last panel) -------------------------
all_axes = findobj(fh, 'Type', 'Axes');
if ~isempty(all_axes)
    legend(all_axes(1), {'Hind leg spring', 'Front leg spring', 'Spinal spring'}, ...
        'FontSize', 7, 'FontName', 'Times New Roman', 'Box', 'off', ...
        'Location', 'best');
end

% ---- Save figure ----------------------------------------
savefig(fh, OUT_FIG);
try
    exportgraphics(fh, OUT_PDF, 'ContentType', 'vector');
catch
    print(fh, OUT_PDF, '-dpdf', '-vector');
end

fprintf('Saved: %s\n', OUT_FIG);
fprintf('Saved: %s\n', OUT_PDF);