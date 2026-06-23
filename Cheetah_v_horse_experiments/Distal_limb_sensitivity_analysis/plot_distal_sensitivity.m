% =========================================================
% plot_distal_sensitivity.m
%
% Plots total cost vs normalized spring stiffness for three
% cases at 12 m/s, each showing:
%   - Nominal result  (original folder structure)
%   - +25% distal limb spring stiffness
%   - -25% distal limb spring stiffness
%
% Three-panel figure: cheetah rotary, horse rotary, horse
% transverse (left to right, all at 12 m/s).
%
% Original data folder structure:
%   <ORIG_BASE>/<OrigFolder>/<speed_ms>/  e.g. Cheetah_Rotary/12_ms/
%
% Sensitivity data folder structure:
%   <SENS_BASE>/<SensFolder>/25_increase/
%   <SENS_BASE>/<SensFolder>/25_decrease/
%
% OUTPUT: distal_sensitivity.fig
%         distal_sensitivity.pdf
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

ORIG_BASE = '..';       % root where original folders live
SENS_BASE = 'Distal_limb_sensitivity_analysis';

SPEED_MS  = 12;

% { orig_folder, sens_folder, species, mass, panel_title }
CASES = {
    'Cheetah_Rotary',   'Cheetah_rotary_12ms',   'cheetah', 45.5,  'Cheetah Rotary';
    'Horse_Rotary',     'Horse_rotary_12ms',      'horse',   545.58, 'Horse Rotary';
    'Horse_Transverse', 'Horse_transverse_12ms',  'horse',   545.58, 'Horse Transverse';
};

PANEL_LABELS = {'A', 'B', 'C'};

OUT_FIG = 'distal_sensitivity.fig';
OUT_PDF = 'distal_sensitivity.pdf';

% Colours
CLR_NOM  = [0.00, 0.45, 0.70];   % blue   — nominal
CLR_INC  = [0.85, 0.33, 0.10];   % red    — +25%
CLR_DEC  = [0.00, 0.62, 0.45];   % green  — -25%

% =========================================================
% ---- FIGURE SETUP ----------------------------------------
% =========================================================

fh = figure('Units', 'centimeters', 'Position', [2 2 22 8]);
set(fh, 'Color', 'w');

% Manual panel positions for a 1x3 layout
left_margins = [0.07, 0.39, 0.71];
bot_margin   = 0.15;
panel_w      = 0.26;
panel_h      = 0.75;

% =========================================================
% ---- MAIN LOOP -------------------------------------------
% =========================================================

for pi = 1:size(CASES, 1)

    orig_folder = CASES{pi, 1};
    sens_folder = CASES{pi, 2};
    mass        = CASES{pi, 4};
    ttl         = CASES{pi, 5};

    % ---- Folder paths -----------------------------------
    orig_path = fullfile(ORIG_BASE, orig_folder, sprintf('%d_ms', SPEED_MS));
    inc_path  = fullfile(ORIG_BASE, SENS_BASE, sens_folder, '25_increase');
    dec_path  = fullfile(ORIG_BASE, SENS_BASE, sens_folder, '25_decrease');

    % ---- Extract data from one folder -------------------
    [k_nom, c_nom] = extract_cost(orig_path, mass);
    [k_inc, c_inc] = extract_cost(inc_path,  mass);
    [k_dec, c_dec] = extract_cost(dec_path,  mass);

    % ---- Create panel -----------------------------------
    ax = axes('Position', [left_margins(pi), bot_margin, panel_w, panel_h]);
    hold(ax, 'on'); box(ax, 'on');

    % Plot lines
    h_nom = plot_curve(ax, k_nom, c_nom, CLR_NOM, '-');
    h_inc = plot_curve(ax, k_inc, c_inc, CLR_INC, '--');
    h_dec = plot_curve(ax, k_dec, c_dec, CLR_DEC, ':');

    % Axis formatting
    set(ax, 'XScale', 'log', 'XLim', [0.08, 150], ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'FontSize', 8, 'FontName', 'Times New Roman', ...
        'TickDir', 'out');

    xlabel(ax, 'Normalized spring stiffness', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, 'Total cost', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax, sprintf('%s)  %s  —  12 m/s', PANEL_LABELS{pi}, ttl), ...
        'FontSize', 9, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

    % Legend on first panel only
    if pi == 1
        valid_h = [h_nom, h_inc, h_dec];
        valid_h = valid_h(isgraphics(valid_h));
        if ~isempty(valid_h)
            legend(ax, valid_h, {'Nominal', '+25% distal k', '-25% distal k'}, ...
                'FontSize', 7, 'FontName', 'Times New Roman', ...
                'Box', 'off', 'Location', 'best');
        end
    end

    hold(ax, 'off');

end

% ---- Save -----------------------------------------------
savefig(fh, OUT_FIG);
try
    exportgraphics(fh, OUT_PDF, 'ContentType', 'vector');
catch
    print(fh, OUT_PDF, '-dpdf', '-vector');
end

fprintf('Saved: %s\n', OUT_FIG);
fprintf('Saved: %s\n', OUT_PDF);

% =========================================================
% ---- LOCAL FUNCTIONS ------------------------------------
% =========================================================

function [k_norm_sorted, costs_sorted] = extract_cost(folder_path, mass)
% Loads all *_k_*.mat files in folder_path, parses normalised
% stiffness from the filename, extracts Data.Cost.Total,
% and returns sorted arrays (ascending stiffness).
% Returns empty arrays if the folder is missing or has no data.

    k_norm_sorted  = [];
    costs_sorted   = [];

    if ~isfolder(folder_path)
        warning('Folder not found: %s', folder_path);
        return
    end

    mat_files = dir(fullfile(folder_path, '*_k_*.mat'));
    if isempty(mat_files)
        % Try without the prefix pattern as fallback
        mat_files = dir(fullfile(folder_path, '*.mat'));
    end
    if isempty(mat_files)
        warning('No .mat files found in: %s', folder_path);
        return
    end

    n = numel(mat_files);
    k_raw = nan(1, n);
    c_raw = nan(1, n);

    for fi = 1:n
        fname = mat_files(fi).name;
        fpath = fullfile(folder_path, fname);

        % Parse stiffness from filename
        tok = regexp(fname, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if isempty(tok)
            warning('  Cannot parse stiffness from: %s', fname);
            continue
        end
        k_raw(fi) = str2double(tok{1}) / mass;

        % Load cost
        try
            S = load(fpath);
            if isfield(S, 'Data') && isfield(S.Data, 'Cost')
                c_raw(fi) = S.Data.Cost.Total;
            else
                warning('  Missing Data.Cost in: %s', fname);
            end
        catch ME
            warning('  Load error in %s: %s', fname, ME.message);
        end
    end

    % Remove failed entries and sort
    valid = ~isnan(k_raw) & ~isnan(c_raw);
    [k_norm_sorted, idx] = sort(k_raw(valid));
    c_tmp = c_raw(valid);
    costs_sorted = c_tmp(idx);

    fprintf('  %s: %d valid points loaded.\n', folder_path, numel(k_norm_sorted));
end


function lh = plot_curve(ax, k, c, clr, lstyle)
% Plots a cost curve and marks its minimum.
% Returns the line handle for legend use.

    lh = gobjects(1);   % default invalid handle

    if isempty(k) || isempty(c)
        return
    end

    lh = plot(ax, k, c, lstyle, 'Color', clr, 'LineWidth', 1.5);

    % Mark minimum
    [c_min, min_idx] = min(c);
    plot(ax, k(min_idx), c_min, 'o', ...
        'MarkerFaceColor', clr, 'MarkerEdgeColor', 'w', ...
        'MarkerSize', 6, 'LineWidth', 0.8);
end