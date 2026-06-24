%% run_Quadri_Victor_combination_PID_R2025a.m
clear
clc
close all
rootDir = "C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models";
addpath(fullfile(rootDir,"params"), fullfile(rootDir,"build"), fullfile(rootDir,"run"));
run(fullfile(rootDir,"params","victor_pid_params.m"));
resultDir = fullfile(rootDir,"results");
if ~exist(resultDir,'dir'), mkdir(resultDir); end
if ~isfile(fullfile(rootDir,"models","Quadri_Victor_combination_PID_R2025a.slx"))
    build_victor_pid_R2025a;
end
load_system(fullfile(rootDir,"models","Quadri_Victor_combination_PID_R2025a.slx"));
simOut = sim("Quadri_Victor_combination_PID_R2025a");
save(fullfile(resultDir,"Quadri_Victor_combination_PID_R2025a_simout.mat"), "simOut");
open_system("Quadri_Victor_combination_PID_R2025a");
