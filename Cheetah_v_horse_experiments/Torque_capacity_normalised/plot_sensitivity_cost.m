% =========================================================
% plot_sensitivity_cost.m
%
% Plots Cost (from Data.Cost in .mat files) vs normalized
% spring stiffness for sensitivity analysis runs at 12 m/s.
%
% Four panels: cheetah_rotary, horse_rotary, cheetah_transverse,
% horse_transverse, one line per panel with minimum marked.
%
% Stiffness is extracted from filenames (pattern: k_value_k_*)
% and normalized by species mass.
%
% OUTPUT: sensitivity_cost.fig
%         sensitivity_cost.pdf
% =========================================================

clear; clc;

% =========================================================
% ---- USER CONFIGURATION ---------------------------------
% =========================================================

BASE_DIR = '.';

% { folder_name, species, gait, mass, panel_title }
CASES = {
    'Cheetah_rotary_12ms',    'cheetah', 'rotary',    45.5, 'Cheetah Rotary';
    'Horse_rotary_12ms',      'horse',   'rotary',    545.58, 'Horse Rotary';
    'Cheetah_transverse_12ms','cheetah', 'transverse',45.5, 'Cheetah Transverse';
    'Horse_transverse_12ms',  'horse',   'transverse',545.58, 'Horse Transverse';
};

OUT_FIG = 'sensitivity_cost.fig';
OUT_PDF = 'sensitivity_cost.pdf';

% Line colour (single colour per panel, all at 12 m/s)
LINE_CLR = [0.00, 0.45, 0.70];  % blue

% Panel layout (same as previous 4-panel plots)
left_margin = 0.09; bot_margin = 0.11; h_gap = 0.10; v_gap = 0.12;
panel_w = (1 - left_margin - 0.03 - h_gap) / 2;
panel_h = (1 - bot_margin - 0.06 - v_gap)  / 2;

panel_left = [left_margin, left_margin + panel_w + h_gap, ...
              left_margin, left_margin + panel_w + h_gap];
panel_bot  = [bot_margin + panel_h + v_gap, bot_margin + panel_h + v_gap, ...
              bot_margin, bot_margin];

% Panel labels
PANEL_LABELS = {'A', 'B', 'C', 'D'};

% =========================================================
% ---- FIGURE SETUP ----------------------------------------
% =========================================================

fh = figure('Units', 'centimeters', 'Position', [2 2 18 14]);
set(fh, 'Color', 'w');

% =========================================================
% ---- MAIN LOOP -------------------------------------------
% =========================================================

for pi = 1:size(CASES,1)

    folder_name = CASES{pi, 1};
    species     = CASES{pi, 2};
    gait        = CASES{pi, 3};
    mass        = CASES{pi, 4};
    ttl         = CASES{pi, 5};

    folder_path = fullfile(BASE_DIR, folder_name);

    if ~isfolder(folder_path)
        warning('Folder not found: %s — skipping panel %d.', folder_path, pi);
        nexttile; title(sprintf('MISSING: %s', ttl)); 
        continue
    end

    mat_files = dir(fullfile(folder_path, '*.mat'));

    if isempty(mat_files)
        warning('No .mat files in %s — skipping panel %d.', folder_path, pi);
        nexttile; title(sprintf('NO FILES: %s', ttl)); 
        continue
    end

    fprintf('Processing %s (%d files)...\n', folder_name, numel(mat_files));

    % ---- Extract stiffness and cost from each .mat --------
    n_files = numel(mat_files);
    stiffness_abs  = nan(1, n_files);
    stiffness_norm = nan(1, n_files);
    cost_vals      = nan(1, n_files);
    valid_mask     = false(1, n_files);

    for fi = 1:n_files
        fname = mat_files(fi).name;
        fpath = fullfile(folder_path, fname);

        % Parse stiffness from filename
        tok = regexp(fname, '^(-?[\d.]+)_k_', 'tokens', 'once');
        if isempty(tok)
            warning('  Could not parse stiffness from: %s', fname);
            continue
        end

        stiffness_abs(fi) = str2double(tok{1});
        stiffness_norm(fi) = stiffness_abs(fi) / mass;

        % Load .mat and extract Cost
        try
            S = load(fpath);
            if isfield(S, 'Data') && isfield(S.Data, 'Cost')
                cost_vals(fi) = S.Data.Cost;
                valid_mask(fi) = true;
            else
                warning('  Missing Data.Cost in: %s', fname);
            end
        catch ME
            warning('  Could not load: %s (%s)', fname, ME.message);
        end
    end

    % Keep only valid entries
    k_norm = stiffness_norm(valid_mask);
    costs  = cost_vals(valid_mask);

    if isempty(k_norm)
        warning('No valid data extracted from %s — skipping panel.', folder_name);
        nexttile; title(sprintf('NO DATA: %s', ttl)); 
        continue
    end

    % Sort by stiffness
    [k_norm, sort_idx] = sort(k_norm);
    costs = costs(sort_idx);

    % ---- Plot on this panel -----
    ax = axes('Position', [panel_left(pi), panel_bot(pi), panel_w, panel_h]);
    hold(ax, 'on'); box(ax, 'on');

    % Line
    plot(ax, k_norm, costs, '-', 'Color', LINE_CLR, 'LineWidth', 1.6);

    % Mark minimum
    [cost_min, min_idx] = min(costs);
    plot(ax, k_norm(min_idx), cost_min, 'o', ...
        'MarkerFaceColor', LINE_CLR, 'MarkerEdgeColor', 'w', ...
        'MarkerSize', 6, 'LineWidth', 0.8);

    % Formatting
    set(ax, 'XScale', 'log', 'XLim', [0.08, 150], ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'FontSize', 8, 'FontName', 'Times New Roman', ...
        'TickDir', 'out');

    xlabel(ax, 'Normalized spring stiffness', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, 'Torque Normalized Cost', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax, sprintf('%s)  %s', PANEL_LABELS{pi}, ttl), ...
        'FontSize', 9, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

    hold(ax, 'off');

    fprintf('  Panel %d: %d valid points, min cost = %.4f at k = %.4f\n', ...
        pi, numel(k_norm), cost_min, k_norm(min_idx));

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