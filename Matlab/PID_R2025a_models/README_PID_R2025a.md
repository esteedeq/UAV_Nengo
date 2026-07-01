# Victor PID R2025a Simulink Layer

This folder contains the corrected PID-converted R2025a Simulink benchmark layer for the four Victor baseline models.

The current generated models follow the corrected missions:

- `Modelo_1gdl_PID_R2025a`: vertical 1-DOF hover/altitude control over `z`, not `theta`.
- `Quadri_Sub_Por_Partes_PID_R2025a`: separated cascaded PID model using the Victor sigmoid references.
- `Quadri_Victor_combination_PID_R2025a`: combined cascaded PID model using the same Victor sigmoid references.
- `Quadrotor_model_2020_PID_R2025a`: 2020 cascaded PID model using the sinusoidal x/y and piecewise z schedule from the source code.

## Build

```matlab
cd('C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models')
addpath('params','build','run')
build_victor_pid_R2025a
```

## Run All

```matlab
cd('C:\00_Research_\Neuromorphic_Embodied_AI\02_Coaxial_TriRotor_Nengo\baseline_victor\PID_R2025a_models')
addpath('params','build','run')
run_all_victor_pid_R2025a
```

`run_all_victor_pid_R2025a` rebuilds the models, simulates all four cases, saves `*_simout.mat`, and then calls `analyze_victor_pid_results` to regenerate the CSV files, plots, and markdown report in `results/`.

## Models

Generated models are saved in `models/`:

- `Modelo_1gdl_PID_R2025a.slx` - 1-DOF hover/altitude PID control for `z`.
- `Quadri_Sub_Por_Partes_PID_R2025a.slx` - separated cascaded position/attitude PID control with sigmoid `x`, `y`, and `z` references.
- `Quadri_Victor_combination_PID_R2025a.slx` - combined cascaded PID control with the same Victor sigmoid mission.
- `Quadrotor_model_2020_PID_R2025a.slx` - 2020 quadrotor cascaded PID control with `sin(0.1t)` x/y references and the piecewise z profile.

## Parameters

Tune all PID gains and limits in:

```matlab
params/victor_pid_params.m
```

## Documentation and Diagrams

- Equations: `docs/*.md`
- Draw.io schematics: `diagrams/*.drawio`
- Source/mission traceability: `docs/source_truth_map_pid_R2025a.md`
- Meeting summary: `docs/team_meeting_brief_pid_R2025a.md`
- Latest benchmark report: `results/PID_ANALYSIS_REPORT.md`

## Notes

The original PD/open-loop models are not modified. These are new R2025a-native PID conversion models generated from MATLAB scripts.

For the 1GDL case, the old extracted `Modelo_1gdl.slx` inventory contains angular/theta dynamics, but this PID benchmark layer intentionally follows the corrected hover mission used for this work: altitude `z` control from the `Quad1Z.py`-style vertical plant. The two local Victor quadrotor models follow the source sigmoid missions, not the earlier draft sinusoidal/circular references.
