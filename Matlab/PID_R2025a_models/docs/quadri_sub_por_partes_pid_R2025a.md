# Quadri_Sub_Por_Partes_PID_R2025a


Source model: `Quadri_Sub_Por_Partes.slx`.

Purpose: convert the separated Victor quadrotor PD sections into a cascaded PID model while keeping the same plant equations.

Generated Simulink model:

```text
models/Quadri_Sub_Por_Partes_PID_R2025a.slx
```

The Simulink diagram keeps the controller concept separated into reference generation, PID controller, and RK4 plant blocks. The controller block internally implements the separated position PID and attitude PID sections.

## Reference Mission

The original Victor model uses sigmoid reference generators for the channel missions, plus Nengo-generated `sim_data_*` traces for comparison. The PID benchmark reproduces the deterministic reference missions:

$$
z_d(t)=\frac{3}{1+\exp[-(t-4)]},
$$

$$
x_d(t)=\frac{1}{1+\exp[-(t-5)]},
$$

$$
y_d(t)=\frac{1.8}{1+\exp[-(t-5)]},
$$

$$
\psi_d=0.
$$

The x and y derivative references are computed analytically from the sigmoid curves. The source z-thrust Fcn block damps measured vertical velocity directly, so the z thrust law uses `-dz` rather than `dz_d-dz`.

For each controlled channel, the PD law was converted to PID by adding an integral state:

$$
I_j(k+1)=\mathrm{clip}\left(I_j(k)+\Delta t\,e_j(k),-I_{j,\max},I_{j,\max}\right).
$$

Outer-loop position PID:

$$
e_x=x_d-x,\quad \dot{e}_x=\dot{x}_d-\dot{x},
$$

$$
a_x=K_{p,x}e_x+K_{i,x}I_x+K_{d,x}\dot{e}_x,
$$

$$
e_y=y_d-y,\quad \dot{e}_y=\dot{y}_d-\dot{y},
$$

$$
a_y=K_{p,y}e_y+K_{i,y}I_y+K_{d,y}\dot{e}_y,
$$

$$
e_z=z_d-z,\quad \dot{e}_z=-\dot{z},
$$

$$
u_z=K_{p,z}e_z+K_{i,z}I_z-K_{d,z}\dot{z}.
$$

Desired attitude and thrust:

$$
\theta_d=\mathrm{clip}\left(\operatorname{atan2}(a_x,g),-\theta_{\max},\theta_{\max}\right),
$$

$$
\phi_d=\mathrm{clip}\left(\operatorname{atan2}(-a_y\cos\theta,g),-\phi_{\max},\phi_{\max}\right),
$$

$$
u=\mathrm{clip}\left(\frac{mg+u_z}{\max(\cos\theta\cos\phi,d_{\min})},u_{\min},u_{\max}\right).
$$

Inner-loop attitude PID:

$$
\tau_\psi=K_{p,\psi}e_\psi+K_{i,\psi}I_\psi+K_{d,\psi}\dot{e}_\psi,
$$

$$
\tau_\theta=K_{p,\theta}(\theta_d-\theta)+K_{i,\theta}I_\theta-K_{d,\theta}\dot{\theta},
$$

$$
\tau_\phi=K_{p,\phi}(\phi_d-\phi)+K_{i,\phi}I_\phi-K_{d,\phi}\dot{\phi}.
$$

The state order is:

```text
X = [x, dx, y, dy, z, dz, psi, dpsi, theta, dtheta, phi, dphi]^T
```

The PID-controlled quadrotor dynamics use the Victor model:

$$
\dot{x}=\dot{x},
$$

$$
\ddot{x}=\frac{u}{m}\left(\sin\phi\sin\psi+\cos\phi\cos\psi\sin\theta\right),
$$

$$
\dot{y}=\dot{y},
$$

$$
\ddot{y}=\frac{u}{m}\left(\cos\phi\sin\theta\sin\psi-\cos\psi\sin\phi\right),
$$

$$
\dot{z}=\dot{z},
$$

$$
\ddot{z}=\frac{u\cos\theta\cos\phi}{m}-g,
$$

$$
\dot{\psi}=\dot{\psi},\quad \ddot{\psi}=\tau_\psi,
$$

$$
\dot{\theta}=\dot{\theta},\quad \ddot{\theta}=\tau_\theta,
$$

$$
\dot{\phi}=\dot{\phi},\quad \ddot{\phi}=\tau_\phi.
$$

The plant block advances this model with RK4:

$$
X_{k+1}=X_k+\frac{\Delta t}{6}(k_1+2k_2+2k_3+k_4),
$$

where each $k_i$ evaluates the same continuous dynamics with the current command held constant over the sample.

## Simulink Implementation Note

The generated Simulink model uses two explicit discrete memory blocks:

```text
State Memory         stores X(k)
PID Integral Memory  stores I(k)
```

The controller block receives the current integral state and outputs the clipped next integral state. Parameters are supplied through a `Parameter Vector` Constant block built from `params/victor_pid_params.m`. Edit that parameter file, then rebuild the models to propagate changed gains or limits.

