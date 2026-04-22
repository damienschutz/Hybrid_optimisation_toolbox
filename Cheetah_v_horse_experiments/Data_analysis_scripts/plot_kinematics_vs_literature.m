% =========================================================
% plot_kinematics_vs_literature.m
%
% Reads cheetah_rotary_kinematics.csv and produces a 3-panel
% figure comparing simulation results against literature.
%
% Panel A: Stride length vs speed
% Panel B: Stride frequency vs speed
% Panel C: Forelimb duty factor vs speed
%
% Simulation is shown as a shaded range (min-max across all
% representative stiffness values) plus a central line at
% the median. Literature values are shown as markers.
%
% OUTPUT: kinematics_vs_literature.fig
%         kinematics_vs_literature.pdf
% =========================================================

clear; clc;

% ---- INPUT ----------------------------------------------
IN_CSV  = 'cheetah_rotary_kinematics.csv';
OUT_FIG = 'kinematics_vs_literature.fig';
OUT_PDF = 'kinematics_vs_literature.pdf';
% ---------------------------------------------------------

% ---- Load CSV -------------------------------------------
if ~isfile(IN_CSV)
    error('Cannot find %s — make sure it is in the current folder.', IN_CSV);
end
T = readtable(IN_CSV, 'TextType', 'string');

% Check for expected columns (readtable sanitises / -> _)
disp('Columns found:');
disp(T.Properties.VariableNames');

% ---- Speeds to plot -------------------------------------
SPEEDS = [8, 10, 12, 15];

% ---- Colours --------------------------------------------
CLR_SIM = [0.00, 0.45, 0.70];   % blue  — simulation
CLR_LIT = [0.85, 0.33, 0.10];   % red   — literature

% =========================================================
% ---- Compute per-speed simulation ranges ----------------
% =========================================================

% For each speed, collect the simulated values across all
% stiffness entries, then take min, median and max.

sim_stride_lo  = nan(size(SPEEDS));  sim_stride_med = nan(size(SPEEDS));  sim_stride_hi  = nan(size(SPEEDS));
sim_freq_lo    = nan(size(SPEEDS));  sim_freq_med   = nan(size(SPEEDS));  sim_freq_hi    = nan(size(SPEEDS));
sim_df_lo      = nan(size(SPEEDS));  sim_df_med     = nan(size(SPEEDS));  sim_df_hi      = nan(size(SPEEDS));

lit_stride = nan(size(SPEEDS));
lit_freq   = nan(size(SPEEDS));
lit_df     = nan(size(SPEEDS));

for si = 1:numel(SPEEDS)
    spd  = SPEEDS(si);
    rows = T.target_speed_ms == spd;

    if ~any(rows)
        warning('No rows found for speed %d m/s', spd);
        continue
    end

    sub = T(rows, :);

    % Simulation ranges
    sim_stride_lo(si)  = min(sub.stride_length_m);
    sim_stride_med(si) = median(sub.stride_length_m);
    sim_stride_hi(si)  = max(sub.stride_length_m);

    sim_freq_lo(si)    = min(sub.stride_frequency_Hz);
    sim_freq_med(si)   = median(sub.stride_frequency_Hz);
    sim_freq_hi(si)    = max(sub.stride_frequency_Hz);

    sim_df_lo(si)      = min(sub.duty_factor_FR);
    sim_df_med(si)     = median(sub.duty_factor_FR);
    sim_df_hi(si)      = max(sub.duty_factor_FR);

    % Literature values (same for every stiffness row at this speed)
    lit_stride(si) = sub.lit_stride_length_m(1);
    lit_freq(si)   = sub.lit_stride_frequency_Hz(1);
    lit_df(si)     = sub.lit_duty_factor(1);
end

% =========================================================
% ---- Figure ---------------------------------------------
% =========================================================

fh = figure('Units', 'centimeters', 'Position', [2 2 18 7]);
set(fh, 'Color', 'w');

tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---- Panel config ---------------------------------------
panels = { ...
    'A', 'Stride length',   'Stride length (m)', ...
        sim_stride_lo, sim_stride_med, sim_stride_hi, lit_stride;  ...
    'B', 'Stride frequency','Stride frequency (Hz)', ...
        sim_freq_lo,   sim_freq_med,   sim_freq_hi,   lit_freq;    ...
    'C', 'Duty factor (FR)','Duty factor', ...
        sim_df_lo,     sim_df_med,     sim_df_hi,     lit_df;      ...
};

for pi = 1:3

    label   = panels{pi, 1};
    ttl     = panels{pi, 2};
    ylbl    = panels{pi, 3};
    lo      = panels{pi, 4};
    med     = panels{pi, 5};
    hi      = panels{pi, 6};
    lit_val = panels{pi, 7};

    ax = nexttile;
    hold(ax, 'on');

    % Shaded range (min to max across stiffnesses)
    x_patch = [SPEEDS, fliplr(SPEEDS)];
    y_patch = [lo,     fliplr(hi)];
    fill(ax, x_patch, y_patch, CLR_SIM, ...
        'FaceAlpha', 0.25, 'EdgeColor', 'none');

    % Median simulation line
    plot(ax, SPEEDS, med, '-o', ...
        'Color',           CLR_SIM, ...
        'MarkerFaceColor', CLR_SIM, ...
        'MarkerEdgeColor', 'w', ...
        'LineWidth',       1.4, ...
        'MarkerSize',      5);

    % Literature markers
    plot(ax, SPEEDS, lit_val, 's', ...
        'Color',           CLR_LIT, ...
        'MarkerFaceColor', CLR_LIT, ...
        'MarkerEdgeColor', 'w', ...
        'LineWidth',       1.2, ...
        'MarkerSize',      6);

    % Formatting
    set(ax, 'XTick', SPEEDS, 'FontSize', 8, 'FontName', 'Times New Roman', ...
        'Box', 'on', 'TickDir', 'out');
    xlim(ax, [6.5, 16.5]);
    xlabel(ax, 'Speed (m/s)',  'FontSize', 8, 'FontName', 'Times New Roman');
    ylabel(ax, ylbl,           'FontSize', 8, 'FontName', 'Times New Roman');
    title(ax,  [label ')  ' ttl], ...
        'FontSize', 8, 'FontName', 'Times New Roman', 'FontWeight', 'bold');

    % Legend on first panel only
    if pi == 1
        legend(ax, {'Simulation range', 'Simulation median', 'Literature'}, ...
            'FontSize', 7, 'FontName', 'Times New Roman', ...
            'Box', 'off', 'Location', 'northwest');
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