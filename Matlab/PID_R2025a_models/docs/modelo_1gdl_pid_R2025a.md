# Modelo_1gdl_PID_R2025a

Source model: `Modelo_1gdl.slx`.

Corrected purpose: vertical 1-DOF hover/altitude control. The controlled state is `z`, not `theta`.

Generated Simulink model:

```text
models/Modelo_1gdl_PID_R2025a.slx
```

## Source Of Truth

The altitude dynamics and gains follow Victor's z-axis Python/Nengo file:

```text
Codigos-20260530T060343Z-3-001/Codigos/Quad1Z.py
```

That file uses:

$$
m=1,\quad g=9.81,\quad k_{pz}=40,\quad k_{dz}=5,\quad \theta=0,\quad \phi=0.
$$

The replicated Simulink z plant in `R2025a_replica_build/matlab_function_code/Quadri_Victor_combination_MATLAB_Function.m` uses:

$$
\ddot z = \frac{T}{m}\cos\theta\cos\phi-g.
$$

For the 1GDL hover case, the fixed hover attitude is:

$$
\theta=\phi=0,
$$

so:

$$
\ddot z = \frac{T}{m}-g.
$$

## State

$$
x=\begin{bmatrix}z & \dot z\end{bmatrix}^T.
$$

## Reference

The reference block outputs:

$$
r=\begin{bmatrix}z_d & \dot z_d\end{bmatrix}^T.
$$

The default altitude reference matches the smooth sigmoid used in `Quad1Z.py`:

$$
z_d(t)=\frac{A}{1+\exp[-k(t-t_c)]}.
$$

With the default parameters:

$$
A=3,\quad k=1,\quad t_c=4.
$$

The derivative reference is computed analytically:

$$
\dot z_d(t)=\frac{A k \exp[-k(t-t_c)]}{(1+\exp[-k(t-t_c)])^2}.
$$

## PID Hover Controller

Altitude error:

$$
e_z=z_d-z.
$$

Derivative error:

$$
\dot e_z=\dot z_d-\dot z.
$$

Integral error with anti-windup clipping:

$$
I_z(k+1)=\mathrm{clip}(I_z(k)+\Delta t\,e_z(k),-I_{\max},I_{\max}).
$$

The raw thrust command is:

$$
T_\mathrm{raw}=K_p e_z+K_i I_z+K_d\dot e_z+mg.
$$

The feedforward term `mg` holds hover when the tracking error is zero. The applied thrust is saturated:

$$
T=\mathrm{clip}(T_\mathrm{raw},T_{\min},T_{\max}).
$$

Default values:

$$
K_p=40,\quad K_i=2,\quad K_d=5,\quad T_{\min}=0,\quad T_{\max}=3mg.
$$

## Plant

The vertical hover plant is:

$$
\dot x =
\begin{bmatrix}
\dot z \\
\frac{T}{m}\cos\theta\cos\phi-g
\end{bmatrix}.
$$

For this 1GDL hover model:

$$
\theta=0,\quad \phi=0,
$$

therefore:

$$
\dot x =
\begin{bmatrix}
\dot z \\
\frac{T}{m}-g
\end{bmatrix}.
$$

The plant is advanced with RK4 at:

```text
pid_dt = 0.001 s
```

## Simulink Implementation Note

The generated Simulink model uses two explicit discrete memory blocks:

```text
Altitude State Memory         stores X(k) = [z(k), dz(k)]
Altitude PID Integral Memory  stores I_z(k)
```

The controller block receives the current integral state and outputs the clipped next integral state. Parameters are supplied through a `Parameter Vector` Constant block built from:

```text
params/victor_pid_params.m
```

Edit that parameter file, then rebuild the models to propagate changed gains, reference settings, or saturation limits.
