%% run_Quadrotor_model_2020_PID_R2025a.m
clear
clc
close all
rootDir = "C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models";
addpath(fullfile(rootDir,"params"), fullfile(rootDir,"build"), fullfile(rootDir,"run"));
run(fullfile(rootDir,"params","victor_pid_params.m"));
resultDir = fullfile(rootDir,"results");
if ~exist(resultDir,'dir'), mkdir(resultDir); end
if ~isfile(fullfile(rootDir,"models","Quadrotor_model_2020_PID_R2025a.slx"))
    build_victor_pid_R2025a;
end
load_system(fullfile(rootDir,"models","Quadrotor_model_2020_PID_R2025a.slx"));
simOut = sim("Quadrotor_model_2020_PID_R2025a");
save(fullfile(resultDir,"Quadrotor_model_2020_PID_R2025a_simout.mat"), "simOut");
open_system("Quadrotor_model_2020_PID_R2025a");
