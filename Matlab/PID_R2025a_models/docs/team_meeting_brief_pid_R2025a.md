# Team Meeting Brief: PID R2025a Simulink Benchmark Layer

## One-Sentence Summary

We added a reproducible MATLAB/Simulink R2025a benchmark layer for the Victor UAV baseline models, converting the original PD/open-loop style models into PID-controlled Simulink models with scripts, documentation, diagrams, and benchmark results.

## Why We Built It

The goal is to compare MATLAB/Simulink PID baselines against the Python/Nengo/NEF UAV simulations using the same missions, plant conventions, controller signals, and logged outputs.

## What Is Included

- `params/`: one parameter file for gains, limits, masses, gravity, references, and simulation time.
- `build/`: MATLAB scripts that programmatically generate the Simulink models.
- `build/matlab_function_code/`: source code used inside MATLAB Function blocks.
- `models/`: generated R2025a `.slx` files.
- `run/`: scripts to run one model or all models.
- `results/`: regenerated benchmark `.mat`, `.csv`, `.png`, and summary report files.
- `docs/`: model-by-model equations, source traceability, and this meeting brief.
- `diagrams/`: draw.io schematics matching the generated Simulink layouts.

## The Four Models

1. `Modelo_1gdl_PID_R2025a`

   This is the corrected 1-DOF hover/altitude benchmark. The controlled state is `z`, not `theta`.

2. `Quadri_Sub_Por_Partes_PID_R2025a`

   This is a separated cascaded PID version of the Victor quadrotor model using the original sigmoid position missions.

3. `Quadri_Victor_combination_PID_R2025a`

   This is the combined cascaded PID version using the same Victor sigmoid mission and plant conventions.

4. `Quadrotor_model_2020_PID_R2025a`

   This follows the 2020 reference schedule: sinusoidal x/y motion and a piecewise z profile.

## Corrected Mission Details

For the two Victor quadrotor models:

```math
z_d(t)=\frac{3}{1+\exp[-(t-4)]}
```

```math
x_d(t)=\frac{1}{1+\exp[-(t-5)]}
```

```math
y_d(t)=\frac{1.8}{1+\exp[-(t-5)]}
```

```math
\psi_d(t)=0
```

For the 1GDL hover model:

```math
\ddot z=\frac{T}{m}\cos\theta\cos\phi-g,\qquad \theta=\phi=0
```

The 1GDL controller is:

```math
T=mg+K_p(z_d-z)+K_iI_z+K_d(\dot z_d-\dot z)
```

## How To Run Everything

From MATLAB R2025a:

```matlab
cd('C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models')
addpath('params','build','run')
run_all_victor_pid_R2025a
```

In the GitHub repository, use:

```matlab
cd('Matlab/PID_R2025a_models')
addpath('params','build','run')
run_all_victor_pid_R2025a
```

## What To Show The Team

- Open `README_PID_R2025a.md` for usage.
- Open `docs/source_truth_map_pid_R2025a.md` to show how each model maps back to the source mission.
- Open `results/PID_ANALYSIS_REPORT.md` for the benchmark metrics.
- Open `diagrams/*.drawio` to show the Simulink architecture visually.
- Open `models/*.slx` in MATLAB R2025a to inspect the generated models.

## Key Result Message

The corrected models run without command saturation in the tested benchmark cases. The local Victor quadrotor models now track the intended sigmoid missions, and the 1GDL model is now aligned with the hover/altitude `z` mission.

## Suggested Meeting Explanation

I would explain it like this:

> We created a new R2025a MATLAB/Simulink benchmark layer under `Matlab/PID_R2025a_models`. It does not replace the original Victor or Nengo files. Instead, it gives us reproducible PID Simulink versions of the four baseline models, with scripted builders, saved `.slx` models, draw.io schematics, documented equations, and exported benchmark results. We corrected the 1GDL case to be altitude `z` hover control, and corrected the Victor quadrotor missions to use the original sigmoid references. The project can be rebuilt and rerun from one MATLAB command, which regenerates the models and all result files.
