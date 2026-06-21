% =========================================================
% plot_peak_energy_vs_stiffness.m
%
% For cheetah rotary and horse transverse at 12 m/s, sweeps
% all available spinal stiffness values and extracts:
%   - peak spinal spring energy
%   - peak forelimb spring energy
%   - peak hindlimb spring energy
%
% Produces a 2x2 figure:
%   Top row:    peak elastic energy vs normalised stiffness
%   Bottom row: spinal fraction of total peak energy
%
% Energy formulas (consistent with plot_spring_energy.m):
%   E_spine = (1/2) * k_spine_abs * theta_spine^2
%   E_leg   = (1/2) * k_leg       * (theta_leg - theta_rest)^2
%
% OUTPUT: peak_energy_vs_stiffness.fig
%         peak_energy_vs_stiffness.pdf
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR  = '..';
SPEED_MS  = 12;

% ---- Species parameters (from plot_spring_energy.m) -----
PARAMS.cheetah.leg_k          = 5000;
PARAMS.cheetah.leg_rest_front = 0.25;
PARAMS.cheetah.leg_rest_hind  = 0.25;
PARAMS.cheetah.mass           = 45.5;

PARAMS.horse.leg_k            = 55000;
PARAMS.horse.leg_rest_front   = 1.02;
PARAMS.horse.leg_rest_hind    = 0.71;
PARAMS.horse.mass             = 545.18;

% ---- Cases ----------------------------------------------
% { folder_name, species, gait, panel_title }
CASES = {
    'Cheetah_Rotary',   'cheetah', 'rotary',    'Cheetah rotary — 12 m/s';
    'Horse_Transverse', 'horse',   'transverse','Horse transverse — 12 m/s';
};

% ---- DOF indices ----------------------------------------
DOF_SPINE     = 7;
DOF_LEG_FRONT = 10;
DOF_LEG_HIND  = 19;

% ---- Output files ---------------------------------------
OUT_FIG = 'peak_energy_vs_stiffness.fig';
OUT_PDF = 'peak_energy_vs_stiffness.pdf';

% ---- Colours --------------------------------------------
CLR_SPINE = [0.00, 0.45, 0.70];   % blue
CLR_FRONT = [0.85, 0.33, 0.10];   % vermilian
CLR_HIND  = [0.00, 0.62, 0.45];   % green
CLR_OPT  =  [0.80, 0.47, 0.65];   % purple  — optimal stiffness line

% =========================================================
% ---- READ OPTIMAL STIFFNESS FROM summary_table.csv ------
% =========================================================
% Looks up the normalised stiffness at minimum COT for each
% case (species x gait x speed) from the summary table.
% If the file is missing, optimal lines are simply omitted.

SUMMARY_CSV = 'summary_table.csv';
opt_stiffness = containers.Map('KeyType','char','ValueType','double');

if isfile(SUMMARY_CSV)
    SUM = readtable(SUMMARY_CSV, 'TextType', 'string');
    for ri = 1:height(SUM)
        if SUM.target_speed_ms(ri) ~= SPEED_MS; continue; end
        key = sprintf('%s_%s', SUM.species(ri), SUM.gait_type(ri));
        opt_stiffness(char(key)) = SUM.stiffness_norm_at_min(ri);
    end
    fprintf('Loaded optimal stiffnesses from %s\n', SUMMARY_CSV);
else
    warning('summary_table.csv not found — optimal stiffness lines will be omitted.');
end

% =========================================================
% ---- FIGURE SETUP ---------------------------------------
% =========================================================

fh = figure('Units', 'centimeters', 'Position', [2 2 18 14]);
set(fh, 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); %#ok<NASGU>

% =========================================================
% ---- MAIN LOOP ------------------------------------------
% =========================================================

for ci = 1:size(CASES, 1)

    folder_name = CASES{ci, 1};
    species     = CASES{ci, 2};
    gait        = CASES{ci, 3};  %#ok<NASGU>
    panel_title = CASES{ci, 4};

    p    = PARAMS.(species);
    mass = p.mass;

    speed_folder = fullfile(BASE_DIR, folder_name, sprintf('%d_ms', SPEED_MS));

    if ~isfolder(speed_folder)
        warning('Folder not found: %s — skipping.', speed_folder);
        continue
    end

    mat_files = dir(fullfile(speed_folder, '*_k_*.mat'));
    if isempty(mat_files)
        warning('No .mat files in %s — skipping.', speed_folder);
        continue
    end

    % ---- Build stiffness index & sort -------------------
    n_files    = numel(mat_files);
    avail_norm = nan(1, n_files);
    avail_abs  = nan(1, n_files);

    for fi = 1:n_files
        t = regexp(mat_files(fi).name, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if ~isempty(t)
            avail_abs(fi)  = str2double(t{1});
            avail_norm(fi) = avail_abs(fi) / mass;
        end
    end

    [avail_norm, sort_idx] = sort(avail_norm);
    avail_abs  = avail_abs(sort_idx);
    mat_files  = mat_files(sort_idx);

    % ---- Pre-allocate result vectors --------------------
    peak_spine = nan(1, n_files);
    peak_front = nan(1, n_files);
    peak_hind  = nan(1, n_files);

    % ---- Sweep all stiffness files ----------------------
    for fi = 1:n_files

        fpath = fullfile(speed_folder, mat_files(fi).name);

        try
            S = load(fpath);
            if ~isfield(S, 'Data'); continue; end
            D = S.Data;
        catch
            continue
        end

        % Only use successful solves
        try
            if ~D.Return.success; continue; end
        catch
            continue
        end

        % Reconstruct position traces across all phases
        n_phases = numel(D.PhaseData);
        n_pts    = 9;
        n_total  = n_pts * n_phases;

        spine_pos = zeros(1, n_total);
        front_pos = zeros(1, n_total);
        hind_pos  = zeros(1, n_total);

        count = 1;
        for ph = 1:n_phases
            idx            = count : count + n_pts - 1;
            spine_pos(idx) = D.PhaseData(ph).position(:, DOF_SPINE)';
            front_pos(idx) = D.PhaseData(ph).position(:, DOF_LEG_FRONT)';
            hind_pos(idx)  = D.PhaseData(ph).position(:, DOF_LEG_HIND)';
            count          = count + n_pts;
        end

        % Compute instantaneous energies
        k_spine_abs = avail_abs(fi);

        E_spine = (0.5 * k_spine_abs) .* spine_pos.^2;
        E_front = (0.5 * p.leg_k)     .* (front_pos - p.leg_rest_front).^2;
        E_hind  = (0.5 * p.leg_k)     .* (hind_pos  - p.leg_rest_hind ).^2;

        % Store peak values
        peak_spine(fi) = max(E_spine);
        peak_front(fi) = max(E_front);
        peak_hind(fi)  = max(E_hind);

    end % file loop

    % Remove any NaN entries (failed runs)
    valid = ~isnan(peak_spine) & ~isnan(peak_front) & ~isnan(peak_hind);
    k_plot     = avail_norm(valid);
    ps         = peak_spine(valid);
    pf         = peak_front(valid);
    ph_vals    = peak_hind(valid);

    fprintf('Case %d (%s): %d/%d valid runs\n', ci, folder_name, sum(valid), n_files);

    % ---- Spinal fraction --------------------------------
    total_peak = ps + pf + ph_vals;
    f_spine    = ps ./ total_peak;

    % ==================================================
    % TOP PANEL — peak energy vs normalised stiffness
    % ==================================================
    ax_top = nexttile(ci);
    hold(ax_top, 'on');

    plot(ax_top, k_plot, ps,      '-',  'Color', CLR_SPINE, 'LineWidth', 1.4);
    plot(ax_top, k_plot, pf,      '--', 'Color', CLR_FRONT, 'LineWidth', 1.4);
    plot(ax_top, k_plot, ph_vals, ':',  'Color', CLR_HIND,  'LineWidth', 1.8);

    set(ax_top, 'XScale', 'log', ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'FontSize', 10, 'FontName', 'Times New Roman', ...
        'Box', 'on', 'TickDir', 'out');

    xlabel(ax_top, 'Normalized spinal stiffness', ...
        'FontSize', 10, 'FontName', 'Times New Roman');
    ylabel(ax_top, 'Peak elastic energy (J)', ...
        'FontSize', 10, 'FontName', 'Times New Roman');
    title(ax_top, panel_title, ...
        'FontSize', 10, 'FontName', 'Times New Roman', 'FontWeight', 'bold');

    if ci == 1
        legend(ax_top, {'Spine', 'Forelimb', 'Hindlimb'}, ...
            'FontSize', 9, 'FontName', 'Times New Roman', ...
            'Box', 'off', 'Location', 'northwest');
    end

    % Vertical line at optimal stiffness (min COT)
    opt_key = sprintf('%s_%s', species, CASES{ci,3});
    if isKey(opt_stiffness, opt_key)
        xline(ax_top, opt_stiffness(opt_key), '--', ...
            'Color', CLR_OPT, 'LineWidth', 1.2, ...
            'Label', 'min cost', 'LabelHorizontalAlignment', 'left', ...
            'FontSize', 8, 'FontName', 'Times New Roman');
    end

    hold(ax_top, 'off');

    % ==================================================
    % BOTTOM PANEL — spinal fraction vs normalised stiffness
    % ==================================================
    ax_bot = nexttile(ci + 2);
    hold(ax_bot, 'on');

    plot(ax_bot, k_plot, f_spine, '-', 'Color', CLR_SPINE, 'LineWidth', 1.4);

    % Reference line at f = 1/3 (equal sharing between three springs)
    yline(ax_bot, 1/3, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);

    set(ax_bot, 'XScale', 'log', ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'YLim', [0, 1], ...
        'FontSize', 10, 'FontName', 'Times New Roman', ...
        'Box', 'on', 'TickDir', 'out');

    xlabel(ax_bot, 'Normalized spinal stiffness)', ...
        'FontSize', 10, 'FontName', 'Times New Roman');
    ylabel(ax_bot, 'Spinal fraction of summed peak spring energies', ...
        'FontSize', 10, 'FontName', 'Times New Roman');
    title(ax_bot, [panel_title ' — spinal fraction of summed peak spring energies'], ...
        'FontSize', 10, 'FontName', 'Times New Roman', 'FontWeight', 'bold');

    if ci == 1
        legend(ax_bot, {'f_{spine}', 'Equal sharing (1/3)'}, ...
            'FontSize', 9, 'FontName', 'Times New Roman', ...
            'Box', 'off', 'Location', 'best');
    end

    % Vertical line at optimal stiffness (min COT)
    if isKey(opt_stiffness, opt_key)
        xline(ax_bot, opt_stiffness(opt_key), '--', ...
            'Color', CLR_OPT, 'LineWidth', 1.2, ...
            'Label', 'min cost', 'LabelHorizontalAlignment', 'left', ...
            'FontSize', 8, 'FontName', 'Times New Roman');
    end

    hold(ax_bot, 'off');

end % case loop

% ---- Save -----------------------------------------------
savefig(fh, OUT_FIG);
try
    exportgraphics(fh, OUT_PDF, 'ContentType', 'vector');
catch
    print(fh, OUT_PDF, '-dpdf', '-vector');
end

fprintf('\nSaved: %s\n', OUT_FIG);
fprintf('Saved: %s\n', OUT_PDF);