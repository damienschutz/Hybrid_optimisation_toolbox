% =========================================================
% analyze_torque_effort.m
%
% Reads torque_effort_metrics.csv (and the four master results
% CSVs, for stride_length_m) and produces:
%
%   1. MAIN PLOT  — 4-panel J_tau_norm vs normalised stiffness
%                   (log x-axis), one curve per speed, minima
%                   marked. Same layout/colours as the COT plot.
%
%   2. SUPPORTING PLOT — torque saturation (95th percentile
%                   |tau|/tau_max) vs normalised stiffness for
%                   cheetah rotary, horse rotary, horse
%                   transverse, all at 12 m/s, on one panel.
%
%   3. SUMMARY TABLE — CSV with stiffness at minimum J_tau_norm
%                   and percent reduction/increase relative to
%                   the lowest and highest stiffness tested.
%
% IMPORTANT — J_tau_norm definition:
%   J_tau_norm = Sum_i( (tau_i/tau_i_max)^2 )  /  stride_length_m
%
%   The numerator uses 'sum_tau_norm_sq' from
%   torque_effort_metrics.csv, which is the RAW SUM over all 72
%   collocation points (8 phases x 9 points) and all 13 actuated
%   joints — NOT a true time integral (no dt weighting). Since
%   every trajectory shares the same 72-point collocation
%   structure, this sum is directly comparable across runs, but
%   it is an approximation of integral(Sum(tau/tau_max)^2)dt, not
%   the literal integral. If you need the literal time integral,
%   let me know and I can re-derive it from the .mat files
%   directly using trapz over the true (non-uniform) time vector.
%
% OUTPUTS:
%   torque_effort_main.fig / .pdf
%   torque_saturation_12ms.fig / .pdf
%   torque_effort_summary.csv
% =========================================================

clear; clc;

% ---- INPUT FILES ------------------------------------------
TORQUE_CSV = 'torque_effort_metrics.csv';

MASTER_CSVS = struct( ...
    'cheetah_rotary',     'cheetah_rotary_results.csv', ...
    'cheetah_transverse', 'cheetah_transverse_results.csv', ...
    'horse_rotary',       'horse_rotary_results.csv', ...
    'horse_transverse',   'horse_transverse_results.csv');

OUT_MAIN_FIG = 'torque_effort_main.fig';
OUT_MAIN_PDF = 'torque_effort_main.pdf';
OUT_SAT_FIG  = 'torque_saturation_12ms.fig';
OUT_SAT_PDF  = 'torque_saturation_12ms.pdf';
OUT_SUMMARY  = 'torque_effort_summary.csv';

STIFF_MATCH_TOL = 1e-3;   % tolerance for matching stiffness between CSVs

% ---- Load torque effort CSV --------------------------------
if ~isfile(TORQUE_CSV)
    error('Cannot find %s', TORQUE_CSV);
end
TQ = readtable(TORQUE_CSV, 'TextType', 'string');
TQ = TQ(TQ.solver_success == 1, :);
fprintf('Loaded %d successful torque-metric rows.\n', height(TQ));

% ---- Load all four master CSVs and tag with species_gait ---
master_all = [];
fn = fieldnames(MASTER_CSVS);
for fi = 1:numel(fn)
    fname = MASTER_CSVS.(fn{fi});
    if ~isfile(fname)
        warning('Master CSV not found, skipping: %s', fname);
        continue
    end
    M = readtable(fname, 'TextType', 'string');
    M = M(M.solver_success == 1, :);
    M.species_gait = repmat(string(fn{fi}), height(M), 1);
    master_all = [master_all; M(:, {'species_gait','target_speed_m_s', ...
                                     'stiffness_normalised','stride_length_m'})]; %#ok<AGROW>
end

if isempty(master_all)
    error('No master CSVs loaded — cannot retrieve stride_length_m.');
end

% ---- Merge: attach stride_length_m to each torque row -------
n = height(TQ);
TQ.stride_length_m = nan(n, 1);
TQ.J_tau_norm       = nan(n, 1);

for ri = 1:n
    sg_key = sprintf('%s_%s', TQ.species(ri), TQ.gait(ri));
    mask = master_all.species_gait == sg_key & ...
           master_all.target_speed_m_s == TQ.target_speed_ms(ri);
    if ~any(mask)
        continue
    end
    sub = master_all(mask, :);
    [dist, idx] = min(abs(sub.stiffness_normalised - TQ.stiffness_normalised(ri)));
    if dist <= STIFF_MATCH_TOL
        sl = sub.stride_length_m(idx);
        TQ.stride_length_m(ri) = sl;
        if sl > 0
            TQ.J_tau_norm(ri) = TQ.sum_tau_norm_sq(ri) / sl;
        end
    end
end

n_matched = sum(~isnan(TQ.J_tau_norm));
fprintf('Matched stride length for %d / %d rows.\n', n_matched, n);

% =========================================================
% ---- PLOT 1: MAIN 4-PANEL J_tau_norm vs STIFFNESS ----------
% =========================================================

PANELS = {
    'cheetah', 'rotary',     'Cheetah Rotary';
    'horse',   'rotary',     'Horse Rotary';
    'cheetah', 'transverse', 'Cheetah Transverse';
    'horse',   'transverse', 'Horse Transverse';
};

SPEEDS_MS  = [8, 10, 12, 15];
SPEED_CLRS = [ ...
    0.00, 0.45, 0.70;
    0.85, 0.33, 0.10;
    0.00, 0.62, 0.45;
    0.80, 0.47, 0.65];

fh1 = figure('Units', 'centimeters', 'Position', [2 2 18 14]);
set(fh1, 'Color', 'w');

left_margin = 0.09; bot_margin = 0.11; h_gap = 0.10; v_gap = 0.12;
panel_w = (1 - left_margin - 0.03 - h_gap) / 2;
panel_h = (1 - bot_margin - 0.06 - v_gap)  / 2;
panel_left = [left_margin, left_margin + panel_w + h_gap, ...
              left_margin, left_margin + panel_w + h_gap];
panel_bot  = [bot_margin + panel_h + v_gap, bot_margin + panel_h + v_gap, ...
              bot_margin, bot_margin];

for pi = 1:4
    sp = PANELS{pi,1}; gt = PANELS{pi,2}; ttl = PANELS{pi,3};

    ax = axes('Position', [panel_left(pi), panel_bot(pi), panel_w, panel_h]);
    hold(ax, 'on'); box(ax, 'on');

    mask_panel = TQ.species == sp & TQ.gait == gt;

    legend_handles = gobjects(numel(SPEEDS_MS), 1);
    legend_labels  = cell(numel(SPEEDS_MS), 1);

    for si = 1:numel(SPEEDS_MS)
        spd = SPEEDS_MS(si); clr = SPEED_CLRS(si,:);
        mask = mask_panel & TQ.target_speed_ms == spd;
        sub  = TQ(mask, :);
        if isempty(sub); continue; end

        [~, ord] = sort(sub.stiffness_normalised);
        sub = sub(ord, :);

        k   = sub.stiffness_normalised;
        J   = sub.J_tau_norm;
        valid = ~isnan(k) & ~isnan(J);
        k = k(valid); J = J(valid);
        if isempty(k); continue; end

        lh = plot(ax, k, J, '-', 'Color', clr, 'LineWidth', 1.4);

        [~, min_idx] = min(J);
        plot(ax, k(min_idx), J(min_idx), 'o', ...
            'MarkerFaceColor', clr, 'MarkerEdgeColor', 'w', ...
            'MarkerSize', 6, 'LineWidth', 0.8);

        legend_handles(si) = lh;
        legend_labels{si}  = sprintf('%d m/s', spd);
    end

    set(ax, 'XScale', 'log', 'XLim', [0.08, 150], ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'FontSize', 8, 'FontName', 'Times New Roman', ...
        'TickDir', 'out');

    xlabel(ax, 'Normalised spring stiffness', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, 'J_{\tau,norm}  (\Sigma(\tau/\tau_{max})^2 / stride length)', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax, ttl, 'FontSize', 9, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

    valid_entries = arrayfun(@(h) isgraphics(h) && h ~= 0, legend_handles);
    if any(valid_entries)
        legend(ax, legend_handles(valid_entries), legend_labels(valid_entries), ...
            'FontSize', 7, 'FontName', 'Times New Roman', 'Box', 'off', 'Location', 'best');
    end

    hold(ax, 'off');
end

% savefig(fh1, OUT_MAIN_FIG);
% try
%     exportgraphics(fh1, OUT_MAIN_PDF, 'ContentType', 'vector');
% catch
%     print(fh1, OUT_MAIN_PDF, '-dpdf', '-vector');
% end
% fprintf('Saved: %s and %s\n', OUT_MAIN_FIG, OUT_MAIN_PDF);

% =========================================================
% ---- PLOT 2: TORQUE SATURATION — 4-PANEL --------------------
% Same layout as Plot 1 (species x gait), one line per speed,
% y-axis = 95th percentile |tau|/tau_max, dashed line at 1.0
% marking the torque limit. Only the minimum point per curve
% is marked — the rest is a plain line.
% =========================================================

fh2 = figure('Units', 'centimeters', 'Position', [2 2 18 14]);
set(fh2, 'Color', 'w');

for pi = 1:4
    sp = PANELS{pi,1}; gt = PANELS{pi,2}; ttl = PANELS{pi,3};

    ax = axes('Position', [panel_left(pi), panel_bot(pi), panel_w, panel_h]);
    hold(ax, 'on'); box(ax, 'on');

    mask_panel = TQ.species == sp & TQ.gait == gt;

    legend_handles = gobjects(numel(SPEEDS_MS), 1);
    legend_labels  = cell(numel(SPEEDS_MS), 1);

    for si = 1:numel(SPEEDS_MS)
        spd = SPEEDS_MS(3); clr = SPEED_CLRS(pi,:);
        mask = mask_panel & TQ.target_speed_ms == spd;
        sub  = TQ(mask, :);
        if isempty(sub); continue; end

        [~, ord] = sort(sub.stiffness_normalised);
        sub = sub(ord, :);

        k = sub.stiffness_normalised;
        s = sub.p95_tau_norm;
        valid = ~isnan(k) & ~isnan(s);
        k = k(valid); s = s(valid);
        if isempty(k); continue; end

        % Plain line — no per-point markers
        lh = plot(ax, k, s, '-', 'Color', clr, 'LineWidth', 1.4);

        % Mark only the minimum value
        [~, min_idx] = min(s);
        plot(ax, k(min_idx), s(min_idx), 'o', ...
            'MarkerFaceColor', clr, 'MarkerEdgeColor', 'w', ...
            'MarkerSize', 6, 'LineWidth', 0.8);

        legend_handles(3) = lh;
        legend_labels{3}  = sprintf('%d m/s', spd);
    end

    % Dashed horizontal line at the torque limit (saturation = 1.0)
    yline(ax, 1.0, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8, ...
        'Label', 'torque limit', 'FontSize', 6, 'FontName', 'Times New Roman');

    set(ax, 'XScale', 'log', 'XLim', [0.08, 150], ...
        'XTick', [0.1, 1, 10, 100], ...
        'XTickLabel', {'10^{-1}','10^{0}','10^{1}','10^{2}'}, ...
        'FontSize', 8, 'FontName', 'Times New Roman', ...
        'TickDir', 'out');

    xlabel(ax, 'Normalised spring stiffness', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, '95th percentile |\tau| / \tau_{max}', ...
        'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax, ttl, 'FontSize', 9, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

    valid_entries = arrayfun(@(h) isgraphics(h) && h ~= 0, legend_handles);
    if any(valid_entries)
        legend(ax, legend_handles(valid_entries), legend_labels(valid_entries), ...
            'FontSize', 7, 'FontName', 'Times New Roman', 'Box', 'off', 'Location', 'best');
    end

    hold(ax, 'off');
end

savefig(fh2, OUT_SAT_FIG);
try
    exportgraphics(fh2, OUT_SAT_PDF, 'ContentType', 'vector');
catch
    print(fh2, OUT_SAT_PDF, '-dpdf', '-vector');
end
fprintf('Saved: %s and %s\n', OUT_SAT_FIG, OUT_SAT_PDF);

% =========================================================
% ---- SUMMARY TABLE ------------------------------------------
% =========================================================

sum_header = {'species', 'gait', 'target_speed_ms', ...
    'k_star_at_min', 'J_at_min', ...
    'pct_reduction_from_lowest_k', 'pct_increase_at_highest_k', ...
    'n_runs'};

sfid = fopen(OUT_SUMMARY, 'w');
fprintf(sfid, '%s\n', strjoin(sum_header, ','));

species_list = unique(TQ.species);
for si = 1:numel(species_list)
    sp = species_list(si);
    gait_list = unique(TQ.gait(TQ.species == sp));
    for gi = 1:numel(gait_list)
        gt = gait_list(gi);
        speed_list = unique(TQ.target_speed_ms(TQ.species == sp & TQ.gait == gt));
        for vi = 1:numel(speed_list)
            spd = speed_list(vi);
            mask = TQ.species == sp & TQ.gait == gt & TQ.target_speed_ms == spd;
            sub  = TQ(mask, :);
            sub  = sub(~isnan(sub.J_tau_norm), :);
            if isempty(sub); continue; end

            [~, ord] = sort(sub.stiffness_normalised);
            sub = sub(ord, :);

            J = sub.J_tau_norm;
            k = sub.stiffness_normalised;

            [J_min, min_idx] = min(J);
            k_star = k(min_idx);

            J_low  = J(1);
            J_high = J(end);

            pct_reduction = 100 * (J_min - J_low) / J_low;
            pct_increase  = 100 * (J_high - J_min) / J_min;

            fprintf(sfid, '%s,%s,%g,%g,%g,%.2f,%.2f,%d\n', ...
                sp, gt, spd, k_star, J_min, pct_reduction, pct_increase, height(sub));
        end
    end
end

fclose(sfid);
fprintf('Saved: %s\n', OUT_SUMMARY);