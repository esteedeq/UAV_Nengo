%% run_all_victor_pid_R2025a.m
clear
clc
close all
rootDir = "C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models";
addpath(fullfile(rootDir,"params"), fullfile(rootDir,"build"), fullfile(rootDir,"run"));
run(fullfile(rootDir,"params","victor_pid_params.m"));
build_victor_pid_R2025a;
analyze_victor_pid_results;
fprintf('PID simulations, CSV files, plots, and report saved in %s\n', fullfile(rootDir,"results"));
