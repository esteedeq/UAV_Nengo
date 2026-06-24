# PID R2025a Source Truth Map

This document records the source choices used by the corrected PID R2025a benchmark layer.

## Modelo_1gdl_PID_R2025a

Purpose: vertical 1-DOF hover/altitude control.

Controlled state:

```text
z, dz
```

Reference:

```math
z_d(t)=\frac{3}{1+\exp[-(t-4)]}
```

Controller:

```math
e_z=z_d-z,\qquad \dot e_z=\dot z_d-\dot z
```

```math
T=\mathrm{clip}(mg+K_{p,z}e_z+K_{i,z}I_z+K_{d,z}\dot e_z,T_{\min},T_{\max})
```

Plant:

```math
\ddot z=\frac{T}{m}\cos\theta\cos\phi-g
```

For this hover benchmark:

```math
\theta=0,\qquad \phi=0
```

Source notes:

- The old extracted `Modelo_1gdl.slx` inventory contains angular/theta dynamics.
- This generated PID benchmark intentionally uses the corrected mission for this project: hover/altitude control over `z`.
- The vertical reference and plant convention follow the `Quad1Z.py` / Victor vertical channel style.

## Quadri_Sub_Por_Partes_PID_R2025a

Purpose: separated cascaded quadrotor PID model.

Reference mission:

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

Important controller convention from the source Fcn blocks:

```math
\dot e_z=-\dot z
```

The source z-thrust block damps measured vertical velocity directly; it does not use `dz_ref - dz` for the thrust damping term.

Sources used:

- `Quadri_Sub_Por_Partes.slx`
- extracted Simulink MATLAB Function code in `R2025a_replica_build`
- extracted inventory in `extracted_schematics`

## Quadri_Victor_combination_PID_R2025a

Purpose: combined cascaded quadrotor PID model.

This model uses the same deterministic sigmoid mission as `Quadri_Sub_Por_Partes_PID_R2025a`:

```math
z_d(t)=\frac{3}{1+\exp[-(t-4)]},\quad
x_d(t)=\frac{1}{1+\exp[-(t-5)]},\quad
y_d(t)=\frac{1.8}{1+\exp[-(t-5)]},\quad
\psi_d(t)=0
```

The PID benchmark preserves the same plant and controller sign conventions used by the source Victor blocks:

```math
\theta_d=\operatorname{atan2}(a_x,g)
```

```math
\phi_d=\operatorname{atan2}(-a_y\cos\theta,g)
```

```math
u=\frac{mg+u_z}{\cos\theta\cos\phi}
```

Sources used:

- `Quadri_Victor_combination.slx`
- extracted Simulink MATLAB Function code in `R2025a_replica_build`
- extracted inventory in `extracted_schematics`

## Quadrotor_model_2020_PID_R2025a

Purpose: 2020 quadrotor cascaded PID model.

Reference mission:

```math
x_d(t)=\sin(0.1t),\qquad \dot x_d(t)=0.1\cos(0.1t)
```

```math
y_d(t)=\sin(0.1t),\qquad \dot y_d(t)=0.1\cos(0.1t)
```

```math
z_d(t)=
\begin{cases}
t/10, & 0\le t<10\\
1, & 10\le t<20\\
1-0.05(t-20), & 20\le t<30\\
0.5, & t\ge 30
\end{cases}
```

The source 2020 altitude thrust also damps measured vertical velocity directly:

```math
u_z=K_{p,z}(z_d-z)+K_{i,z}I_z-K_{d,z}\dot z
```

Sources used:

- `Quadrotor_model_2020.slx`
- `Codigos de Matlab R2025a_converted/Quad_control_2026.py`
- `Codigos de Matlab R2025a_replicated/Values_Quadrotor_model.m`

## Shared Generated Files

- Parameters: `params/victor_pid_params.m`
- Builder: `build/build_victor_pid_R2025a.m`
- MATLAB Function block source: `build/matlab_function_code/*.m`
- Run scripts: `run/*.m`
- Results: `results/*.mat`, `results/*.csv`, `results/*.png`, `results/PID_ANALYSIS_REPORT.md`
