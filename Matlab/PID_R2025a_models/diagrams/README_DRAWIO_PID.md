# PID Draw.io Schematics

Open the `.drawio` files in draw.io / diagrams.net.

Each diagram follows the final generated Simulink structure:

```text
Clock + Parameter Vector -> Reference Generator
State Memory + PID Integral Memory + Reference + Parameter Vector -> PID Controller
PID Controller + State Memory + Parameter Vector -> RK4 Plant
RK4 Plant -> State Memory
PID Controller -> PID Integral Memory
Signals -> To Workspace and Scopes
```

Mission labels in the diagrams follow the corrected PID models:

- `Modelo_1gdl_PID_R2025a.drawio`: altitude `z` hover, not theta control.
- `Quadri_Sub_Por_Partes_PID_R2025a.drawio`: Victor sigmoid mission `z->3`, `x->1`, `y->1.8`, `psi=0`.
- `Quadri_Victor_combination_PID_R2025a.drawio`: same Victor sigmoid mission.
- `Quadrotor_model_2020_PID_R2025a.drawio`: 2020 sinusoidal x/y and piecewise z mission.

The original Victor models are not overwritten.
