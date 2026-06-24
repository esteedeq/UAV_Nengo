function analyze_victor_pid_results()
%ANALYZE_VICTOR_PID_RESULTS Run, plot, and analyze the Victor PID R2025a models.
% Outputs are saved in PID_R2025a_models/results.

clearvars -except ans
close all
clc

rootDir = fileparts(fileparts(mfilename('fullpath')));
paramDir = fullfile(rootDir, 'params');
buildDir = fullfile(rootDir, 'build');
modelDir = fullfile(rootDir, 'models');
resultDir = fullfile(rootDir, 'results');
if ~exist(resultDir, 'dir'), mkdir(resultDir); end
addpath(paramDir, buildDir, fullfile(rootDir, 'run'));
run(fullfile(paramDir, 'victor_pid_params.m'));

models = define_models();
trackingRows = {};
commandRows = {};

for k = 1:numel(models)
    cfg = models(k);
    fprintf('Analyzing %s...\n', cfg.name);
    mdlFile = fullfile(modelDir, cfg.name + ".slx");
    if ~isfile(mdlFile)
        build_victor_pid_R2025a;
    end
    load_system(mdlFile);
    out = sim(cfg.name, 'ReturnWorkspaceOutputs', 'on');
    simOut = out;
    save(fullfile(resultDir, cfg.name + "_simout.mat"), 'simOut');

    ref = log_to_matrix(out.get('pid_reference'));
    state = log_to_matrix(out.get('pid_state'));
    cmd = log_to_matrix(out.get('pid_command'));
    err = log_to_matrix(out.get('pid_error'));
    t = ref.time;

    data = struct();
    data.model = cfg.name;
    data.time = t;
    data.reference = ref.values;
    data.state = state.values;
    data.command = cmd.values;
    data.error = err.values;
    data.reference_names = cfg.refNames;
    data.state_names = cfg.stateNames;
    data.command_names = cfg.cmdNames;
    data.error_names = cfg.errNames;
    save(fullfile(resultDir, cfg.name + "_signals.mat"), 'data');

    write_timeseries_csv(fullfile(resultDir, cfg.name + "_timeseries.csv"), t, cfg, ref.values, state.values, cmd.values, err.values);
    plot_model(resultDir, cfg, t, ref.values, state.values, cmd.values, err.values);

    trackingRows = append_tracking_metrics(trackingRows, cfg, t, ref.values, state.values);
    commandRows = append_command_metrics(commandRows, cfg, t, cmd.values);
    close_system(cfg.name, 0);
end

trackingTable = cell2table(trackingRows, 'VariableNames', ...
    {'model','channel','rmse','iae','max_abs_error','final_abs_error','settling_time_s','tolerance'});
commandTable = cell2table(commandRows, 'VariableNames', ...
    {'model','command','min_value','max_value','max_abs_value','mean_value','saturation_pct','lower_limit','upper_limit'});

writetable(trackingTable, fullfile(resultDir, 'pid_tracking_metrics.csv'));
writetable(commandTable, fullfile(resultDir, 'pid_command_metrics.csv'));
save(fullfile(resultDir, 'pid_analysis_tables.mat'), 'trackingTable', 'commandTable');
write_report(fullfile(resultDir, 'PID_ANALYSIS_REPORT.md'), trackingTable, commandTable);

fprintf('\nAnalysis complete. Results written to:\n%s\n', resultDir);
end

function models = define_models()
models = struct([]);

models(1).name = "Modelo_1gdl_PID_R2025a";
models(1).kind = "one_dof";
models(1).refNames = {'z_ref','dz_ref'};
models(1).stateNames = {'z','dz'};
models(1).cmdNames = {'T','T_unsat','T_feedforward','e_z_cmd','I_z_cmd','edot_z_cmd'};
models(1).errNames = {'e_z','I_z','edot_z'};
models(1).trackPairs = [1 1; 2 2];
models(1).trackChannels = {'z','dz'};
models(1).trackTol = [0.05 0.05];
models(1).cmdLimits = [0 29.43; NaN NaN; NaN NaN; NaN NaN; -5 5; NaN NaN];

quadRef = {'x_ref','dx_ref','y_ref','dy_ref','z_ref','dz_ref','psi_ref'};
quadState = {'x','dx','y','dy','z','dz','psi','dpsi','theta','dtheta','phi','dphi'};
quadCmd = {'u','tau_psi','tau_theta','tau_phi','theta_des','phi_des','ax_cmd','ay_cmd','uz_cmd'};
quadErr = {'ex','ey','ez','epsi','etheta','ephi','Ix','Iy','Iz','Ipsi','Itheta','Iphi'};
quadPairs = [1 1; 3 3; 5 5; 7 7];
quadCh = {'x','y','z','psi'};
quadTol = [0.05 0.05 0.05 deg2rad(2)];
quadLimits = [0 29.43; -20 20; -80 80; -80 80; -deg2rad(35) deg2rad(35); -deg2rad(35) deg2rad(35); NaN NaN; NaN NaN; NaN NaN];

models(2).name = "Quadri_Sub_Por_Partes_PID_R2025a";
models(2).kind = "quad_local";
models(2).refNames = quadRef;
models(2).stateNames = quadState;
models(2).cmdNames = quadCmd;
models(2).errNames = quadErr;
models(2).trackPairs = quadPairs;
models(2).trackChannels = quadCh;
models(2).trackTol = quadTol;
models(2).cmdLimits = quadLimits;

models(3) = models(2);
models(3).name = "Quadri_Victor_combination_PID_R2025a";
models(3).kind = "quad_combined";

models(4) = models(2);
models(4).name = "Quadrotor_model_2020_PID_R2025a";
models(4).kind = "quad_2020";
models(4).cmdLimits = [0 26.487; -20 20; -80 80; -80 80; -deg2rad(35) deg2rad(35); -deg2rad(35) deg2rad(35); NaN NaN; NaN NaN; NaN NaN];
end

function sig = log_to_matrix(logStruct)
t = logStruct.time(:);
V = squeeze(logStruct.signals.values);
if isvector(V)
    V = V(:);
else
    if size(V, 2) == numel(t) && size(V, 1) ~= numel(t)
        V = V.';
    end
end
sig = struct('time', t, 'values', V);
end

function write_timeseries_csv(fileName, t, cfg, ref, state, cmd, err)
T = table(t(:), 'VariableNames', {'time_s'});
T = add_columns(T, ref, strcat("ref_", string(cfg.refNames)));
T = add_columns(T, state, strcat("state_", string(cfg.stateNames)));
T = add_columns(T, cmd, strcat("cmd_", string(cfg.cmdNames)));
T = add_columns(T, err, strcat("err_", string(cfg.errNames)));
writetable(T, fileName);
end

function T = add_columns(T, M, names)
for i = 1:numel(names)
    T.(matlab.lang.makeValidName(names(i))) = M(:, i);
end
end

function rows = append_tracking_metrics(rows, cfg, t, ref, state)
for i = 1:size(cfg.trackPairs, 1)
    rIdx = cfg.trackPairs(i, 1);
    sIdx = cfg.trackPairs(i, 2);
    channel = cfg.trackChannels{i};
    e = ref(:, rIdx) - state(:, sIdx);
    tol = cfg.trackTol(i);
    rows(end+1, :) = {char(cfg.name), channel, rmse(e), trapz(t, abs(e)), max(abs(e)), abs(e(end)), settling_time(t, e, tol), tol}; %#ok<AGROW>
end
end

function rows = append_command_metrics(rows, cfg, t, cmd)
for i = 1:numel(cfg.cmdNames)
    x = cmd(:, i);
    lo = cfg.cmdLimits(i, 1);
    hi = cfg.cmdLimits(i, 2);
    if isnan(lo) || isnan(hi)
        satPct = NaN;
    else
        margin = 1e-9 + 1e-6 * max(1, max(abs([lo hi])));
        satPct = 100 * mean((x <= lo + margin) | (x >= hi - margin));
    end
    rows(end+1, :) = {char(cfg.name), cfg.cmdNames{i}, min(x), max(x), max(abs(x)), trapz(t, x)/(t(end)-t(1)), satPct, lo, hi}; %#ok<AGROW>
end
end

function y = rmse(e)
y = sqrt(mean(e.^2));
end

function ts = settling_time(t, e, tol)
absE = abs(e(:));
maxFuture = flipud(cummax(flipud(absE)));
idx = find(maxFuture <= tol, 1, 'first');
if isempty(idx)
    ts = NaN;
else
    ts = t(idx);
end
end

function plot_model(resultDir, cfg, t, ref, state, cmd, err)
if cfg.kind == "one_dof"
    fig = figure('Visible','off','Color','w','Position',[100 100 1100 750]);
    tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');
    nexttile; plot(t, ref(:,1), 'k--', t, state(:,1), 'b-', 'LineWidth', 1.2); grid on; ylabel('z [m]'); legend('z ref','z','Location','best'); title(char(cfg.name), 'Interpreter','none');
    nexttile; plot(t, err(:,1), 'r-', t, err(:,2), 'm-', t, err(:,3), 'c-', 'LineWidth', 1.1); grid on; ylabel('z error'); legend('e_z','I_z','de_z','Location','best');
    nexttile; plot(t, cmd(:,1), 'b-', t, cmd(:,2), 'k--', t, cmd(:,3), 'g:', 'LineWidth', 1.1); grid on; ylabel('thrust [N]'); xlabel('time [s]'); legend('T','T unsat','mg feedforward','Location','best');
    exportgraphics(fig, fullfile(resultDir, cfg.name + "_analysis.png"), 'Resolution', 180);
    close(fig);
else
    fig = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
    tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');
    nexttile; hold on; plot(t, ref(:,1), 'k--', 'LineWidth', 1.0); plot(t, state(:,1), 'b-', 'LineWidth', 1.0); plot(t, ref(:,3), 'r--', 'LineWidth', 1.0); plot(t, state(:,3), 'm-', 'LineWidth', 1.0); plot(t, ref(:,5), '--', 'Color', [0.0 0.45 0.0], 'LineWidth', 1.0); plot(t, state(:,5), '-', 'Color', [0.0 0.55 0.0], 'LineWidth', 1.0); hold off; grid on; ylabel('position [m]'); title(char(cfg.name), 'Interpreter','none'); legend('x ref','x','y ref','y','z ref','z','Location','best');
    nexttile; plot(t, ref(:,7), 'k--', t, state(:,7), 'b-', t, cmd(:,5), 'r--', t, state(:,9), 'r-', t, cmd(:,6), 'm--', t, state(:,11), 'm-', 'LineWidth', 1.0); grid on; ylabel('attitude [rad]'); legend('\psi ref','\psi','\theta_d','\theta','\phi_d','\phi','Location','best');
    nexttile; plot(t, cmd(:,1), 'b-', t, cmd(:,2), 'k-', t, cmd(:,3), 'r-', t, cmd(:,4), 'm-', 'LineWidth', 1.0); grid on; ylabel('commands'); xlabel('time [s]'); legend('u','tau psi','tau theta','tau phi','Location','best');
    exportgraphics(fig, fullfile(resultDir, cfg.name + "_tracking_commands.png"), 'Resolution', 180);
    close(fig);

    fig = figure('Visible','off','Color','w','Position',[100 100 1200 800]);
    tiledlayout(2,1, 'Padding','compact', 'TileSpacing','compact');
    nexttile; plot(t, err(:,1), t, err(:,2), t, err(:,3), t, err(:,4), 'LineWidth', 1.0); grid on; ylabel('tracking error'); title(sprintf('%s errors', char(cfg.name)), 'Interpreter','none'); legend('e_x','e_y','e_z','e_\psi','Location','best');
    nexttile; plot(t, err(:,7), t, err(:,8), t, err(:,9), t, err(:,10), t, err(:,11), t, err(:,12), 'LineWidth', 1.0); grid on; ylabel('integral states'); xlabel('time [s]'); legend('I_x','I_y','I_z','I_\psi','I_\theta','I_\phi','Location','best');
    exportgraphics(fig, fullfile(resultDir, cfg.name + "_errors_integrals.png"), 'Resolution', 180);
    close(fig);
end
end

function write_report(fileName, trackingTable, commandTable)
fid = fopen(fileName, 'w');
fprintf(fid, '# PID Results Analysis\n\n');
fprintf(fid, 'Generated by `run/analyze_victor_pid_results.m`.\n\n');
fprintf(fid, '## Tracking Summary\n\n');
models = unique(string(trackingTable.model), 'stable');
for m = 1:numel(models)
    model = models(m);
    rows = trackingTable(string(trackingTable.model)==model, :);
    fprintf(fid, '### %s\n\n', char(model));
    for i = 1:height(rows)
        st = rows.settling_time_s(i);
        if isnan(st), stTxt = 'not settled'; else, stTxt = sprintf('%.3f s', st); end
        fprintf(fid, '- `%s`: RMSE %.4g, IAE %.4g, max |e| %.4g, final |e| %.4g, settling %s.\n', ...
            rows.channel{i}, rows.rmse(i), rows.iae(i), rows.max_abs_error(i), rows.final_abs_error(i), stTxt);
    end
    crows = commandTable(string(commandTable.model)==model, :);
    satRows = crows(~isnan(crows.saturation_pct) & crows.saturation_pct > 0, :);
    if isempty(satRows)
        fprintf(fid, '\nNo command saturation detected in channels with configured limits.\n\n');
    else
        fprintf(fid, '\nCommand saturation detected:\n');
        for j = 1:height(satRows)
            fprintf(fid, '- `%s`: %.2f%%%% of samples.\n', satRows.command{j}, satRows.saturation_pct(j));
        end
        fprintf(fid, '\n');
    end
end
fprintf(fid, '## Files\n\n');
fprintf(fid, '- `pid_tracking_metrics.csv`\n');
fprintf(fid, '- `pid_command_metrics.csv`\n');
fprintf(fid, '- `*_timeseries.csv`\n');
fprintf(fid, '- `*_signals.mat`\n');
fprintf(fid, '- `*_analysis.png`, `*_tracking_commands.png`, `*_errors_integrals.png`\n');
fclose(fid);
end
