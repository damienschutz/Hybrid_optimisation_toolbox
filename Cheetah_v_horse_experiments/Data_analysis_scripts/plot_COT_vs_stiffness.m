% =========================================================
% plot_COT_vs_stiffness.m
%
% Reads the four extracted CSVs and produces a 4-panel
% figure of COT vs normalised spinal stiffness (log x-axis)
% matching the layout in COT_fig.pdf.
%
% Panels:
%   Top-left:     Cheetah Rotary
%   Top-right:    Horse Rotary
%   Bottom-left:  Cheetah Transverse
%   Bottom-right: Horse Transverse
%
% Each panel shows one curve per speed, with a filled circle
% marking the minimum COT on each curve.
%
% OUTPUT: COT_vs_stiffness.fig  (editable MATLAB figure)
%         COT_vs_stiffness.pdf  (for quick review)
% =========================================================

clear; clc;

% ---- INPUT FILES ----------------------------------------
CSV_FILES = {
    'cheetah_rotary_results.csv';
    'cheetah_transverse_results.csv';
    'horse_rotary_results.csv';
    'horse_transverse_results.csv';
};

OUT_FIG = 'COT_vs_stiffness.fig';
OUT_PDF = 'COT_vs_stiffness.pdf';
% ---------------------------------------------------------

% ---- PANEL LAYOUT ---------------------------------------
% { species, gait, panel title }
PANELS = {
    'cheetah', 'rotary',    'Cheetah Rotary';
    'horse',   'rotary',    'Horse Rotary';
    'cheetah', 'transverse','Cheetah Transverse';
    'horse',   'transverse','Horse Transverse';
};
PANEL_POS = [1, 2, 3, 4];   % subplot positions (2x2, row-major)
% ---------------------------------------------------------

% ---- COLOURS PER SPEED ----------------------------------
% One colour per speed, consistent across all panels
SPEEDS_MS  = [8, 10, 12, 15];
SPEED_CLRS = [ ...
    0.00, 0.45, 0.70;   %  8 m/s  blue
    0.85, 0.33, 0.10;   % 10 m/s  vermillion
    0.00, 0.62, 0.45;   % 12 m/s  green
    0.80, 0.47, 0.65];  % 15 m/s  purple
% ---------------------------------------------------------

% ---- LOAD DATA ------------------------------------------
all_data = [];
for ci = 1:numel(CSV_FILES)
    if ~isfile(CSV_FILES{ci})
        warning('Not found, skipping: %s', CSV_FILES{ci});
        continue
    end
    T = readtable(CSV_FILES{ci}, 'TextType', 'string');
    all_data = [all_data; T]; %#ok<AGROW>
end

if isempty(all_data)
    error('No data loaded. Make sure the CSV files are in the current folder.');
end

% Keep only successful solves
all_data = all_data(all_data.solver_success == 1, :);
fprintf('Plotting %d successful runs.\n', height(all_data));

% ---- CREATE FIGURE --------------------------------------
fh = figure('Units', 'centimeters', 'Position', [2 2 18 14]);
set(fh, 'Color', 'w');

% Tight subplot spacing
left_margin  = 0.09;
bot_margin   = 0.11;
h_gap        = 0.10;
v_gap        = 0.12;
panel_w      = (1 - left_margin - 0.03 - h_gap) / 2;
panel_h      = (1 - bot_margin - 0.06 - v_gap)  / 2;

% Panel positions [left bottom width height] in normalised units
% Order matches PANELS cell array: CR, HR, CT, HT
panel_left = [left_margin, left_margin + panel_w + h_gap, ...
              left_margin, left_margin + panel_w + h_gap];
panel_bot  = [bot_margin + panel_h + v_gap, bot_margin + panel_h + v_gap, ...
              bot_margin, bot_margin];

for pi = 1:4

    sp = PANELS{pi, 1};   % species
    gt = PANELS{pi, 2};   % gait
    ttl = PANELS{pi, 3};  % title

    ax = axes('Position', [panel_left(pi), panel_bot(pi), panel_w, panel_h]);
    hold(ax, 'on');
    box(ax, 'on');

    mask_panel = all_data.species == sp & all_data.gait_type == gt;

    legend_handles = gobjects(numel(SPEEDS_MS), 1);
    legend_labels  = cell(numel(SPEEDS_MS), 1);
    any_plotted    = false;

    for si = 1:numel(SPEEDS_MS)
        spd  = SPEEDS_MS(si);
        clr  = SPEED_CLRS(si, :);

        mask = mask_panel & all_data.target_speed_m_s == spd;
        sub  = all_data(mask, :);

        if isempty(sub); continue; end

        % Sort by normalised stiffness
        [~, ord] = sort(sub.stiffness_normalised);
        sub = sub(ord, :);

        k    = sub.stiffness_normalised;
        cot  = sub.cost_COT;

        % Remove any NaN rows
        valid = ~isnan(k) & ~isnan(cot);
        k   = k(valid);
        cot = cot(valid);
        if isempty(k); continue; end

        % Plot line
        lh = plot(ax, k, cot, '-', ...
            'Color', clr, ...
            'LineWidth', 1.4);

        % Mark minimum with filled circle
        [~, min_idx] = min(cot);
        plot(ax, k(min_idx), cot(min_idx), 'o', ...
            'MarkerFaceColor', clr, ...
            'MarkerEdgeColor', 'w', ...
            'MarkerSize', 6, ...
            'LineWidth', 0.8);

        legend_handles(si) = lh;
        legend_labels{si}  = sprintf('%d m/s', spd);
        any_plotted = true;
    end

    % Axis formatting
    set(ax, 'XScale', 'log');
    set(ax, 'XLim', [0.08, 150]);
    set(ax, 'XTick', [0.1, 1, 10, 100]);
    set(ax, 'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'});
    set(ax, 'FontSize', 8, 'FontName', 'Times New Roman');
    set(ax, 'TickDir', 'out', 'TickLength', [0.015 0.015]);

    xlabel(ax, 'Normalised spring stiffness (k_{spring}/m)', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, 'COT', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax, ttl, ...
        'FontSize', 9, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

    % Legend — only show entries that were actually plotted
    valid_entries = ~isgraphics(legend_handles, 'axes') & ...
                    arrayfun(@(h) isgraphics(h) && h ~= 0, legend_handles);
    if any_plotted && any(valid_entries)
        lg = legend(ax, legend_handles(valid_entries), ...
                    legend_labels(valid_entries), ...
                    'FontSize', 7, 'FontName', 'Times New Roman', ...
                    'Box', 'off', 'Location', 'best');
    end

    hold(ax, 'off');
end

% ---- SAVE -----------------------------------------------
savefig(fh, OUT_FIG);
exportgraphics(fh, OUT_PDF, 'ContentType', 'vector');

fprintf('Saved editable figure: %s\n', OUT_FIG);
fprintf('Saved PDF:             %s\n', OUT_PDF);