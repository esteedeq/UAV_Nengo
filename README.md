# Coaxial Hexarotor UAV: Neuromorphic Modeling and Control

## 1. Introduction
In this work, we:
1. **Develop a neuromorphic modeling and control framework** for a coaxial hexarotor UAV by systematically mapping the reduced nonlinear vehicle dynamics and cascade flight controllers into recurrent spiking neural networks (SNNs) within the Neural Engineering Framework (NEF) using the Nengo simulator.
2. **Quantitatively validate** the proposed SNN-based controller through trajectory-tracking simulations against a conventional MATLAB/Simulink implementation employing PID controllers in the outer translational loops and PD controllers in the inner attitude loops.
3. **Establish a modular and scalable NEF-based architecture** for implementing flight control systems as interconnected spiking neural populations.

---

## 2. Dynamic Modeling of the Coaxial Hexarotor
In this section, the dynamics of the hexarotor depicted in **Figure 1** are modeled and represented using SNNs. 

[Coaxial hexarotor.pdf](https://github.com/user-attachments/files/32489715/dinamica_hexa3.pdf)

<img width="5528" height="3915" alt="dinamica_hexa3_pages-to-jpg-0001" src="https://github.com/user-attachments/assets/5a04ffcb-dc9e-4a79-b67d-890c4c5388da" />


*Figure 1: Coaxial hexarotor vehicle dynamics configuration.*

To facilitate neural encoding, analysis, and control design, the vehicle dynamics are decomposed into altitude, longitudinal, lateral, and directional subsystems. The translational and rotational equations of motion are first derived using the Newton–Euler formulation and subsequently mapped into an SNN-based representation implemented within the Nengo framework. 

The neural implementation is obtained through the third principle of the NEF, which enables the realization of dynamical systems using recurrently connected neural populations. This neuromorphic representation serves as the foundation for the modeling and control architecture developed in this study.

---

## 3. Control Architecture
The proposed control strategy follows a cascade architecture consisting of an outer-loop position controller and an inner-loop attitude controller. The outer loop generates the desired pitch and roll angles required for translational trajectory tracking, while the inner loop regulates the vehicle attitude by computing the corresponding control moments. Following this architecture, the conventional controllers are first presented, after which their equivalent neural network representations are derived.

### 3.1. Classic Controllers
**Figure 2** presents the proposed classic flight control strategies based on the reduced system modeling derived in Section 2. These controllers are used to stabilize and conduct three-dimensional trajectory tracking of the Hexarotor.

[MATLAB_Classic_flight _control.pdf](https://github.com/user-attachments/files/32489719/Quadri_Victor_combination_PID_R2025a_CE_v3.drawio.pdf)


<img width="2929" height="1175" alt="Quadri_Victor_combination_PID_R2025a_CE_v3 drawio_page-0001" src="https://github.com/user-attachments/assets/2385212c-69f6-456e-a35b-9e4b9985d9fa" />

*Figure 2: Classic flight control strategies proposed for the UAV.*

### 3.2. Neural Controller
The proposed controller is implemented as an SNN within the NEF using the Nengo platform. The translational controllers are implemented as neural PID controllers to achieve accurate trajectory tracking and eliminate steady-state errors. In contrast, the attitude controllers employ neural PD controllers, since integral action is not required for attitude stabilization.

**Figure 3** presents the proposed  visualizes the resulting neural flight-control architecture based on the reduced model

[Nengo_neural_flight _control.pdf](https://github.com/user-attachments/files/32489812/Digrama_UAV_dinamyc_ZXTheta-YPhi-3-1.pdf)

<img width="8483" height="3967" alt="Digrama_UAV_dinamyc_ZXTheta-YPhi-3-1_page-0001" src="https://github.com/user-attachments/assets/14ba6aaa-6912-4557-94c9-4c37b09dffa2" />


*Figure 3: The diagram is structured to show translation subsystems, as well as the orientations of the UAV, in a neural format. Each subsystem consists of interconnected neural networks that perform specific transformations for state-space representation and control law calculations. The flow of information is indicated by arrows, with cyan-colored arrows highlighting the dynamic couplings between the subsystems.*

The parameter `dim` specifies the dimensionality of the vector represented by each neural ensemble, `transform` defines the connection scaling matrix, `radius` determines the input range over which the NEF optimizes the decoding weights, and `synapse` denotes the synaptic filter time constant. The symbol `→` represents a connection between ensembles or nodes. Finally, \(T_{\text{neu}}\) is the neural simulation time step, n is the discrete-time index, and \(t_f\) is the final simulation time.

#### **Algorithm 1:** Neural implementation template for a generic subsystem \(\zeta \in \{z, x, \theta, y, \phi, \psi\}\) with PID/PD control.

```text
Require: Neuronal parameters; state pair and subsystem input υ_ζ

While T_neu * n < t_f do:
    ■ Subsystem dynamics:
        - Ensemble F'_ζ with dim = 2, radius = 5
        - Ensemble G'_ζ with dim = 1, radius = 5
        - G'_ζ → F'_ζ, synapse = τ
        - Call function 'G_fun' to compute G'_ζ = τ [0, υ_ζ]ᵀ
        - F'_ζ → F'_ζ, synapse = τ
        - Call function 'F_fun' to compute F'_ζ = [χ₁ + τ χ₂, χ₂]ᵀ
        
    ■ Tracking error (e_ζ = ζ_d - ζ):
        - Reference → ζ_d, dim = 1  (Mission reference for ζ ∈ {x,y,z,ψ})
        - Ensemble e_ζ with dim = 1, radius = 5
        - ζ_d → e_ζ, transform = 1, synapse = None
        - F'_ζ[0] → e_ζ, transform = -1, synapse = None
        
    ■ Derivative of the tracking error:
        - Ensemble ė_ζ with dim = 1, radius = 5
        - e_ζ → ė_ζ, transform = 1/τ, synapse = τ
        - e_ζ → ė_ζ, transform = -1/τ, synapse = 2τ
        
    ■ Integral of the tracking error (Only for ζ ∈ {x, y, z}):
        for ζ ∈ {x, y, z} do:
            - Ensemble ∫ e_ζ with dim = 1, radius = 1
            - e_ζ → ∫ e_ζ, transform = k_i_ζ * τ, synapse = τ
            - ∫ e_ζ → ∫ e_ζ, transform = 1, synapse = τ
        end for
        
    ■ Control signal:
        - Ensemble u_ζ with dim = 1, radius = 5
        - e_ζ → u_ζ, transform = k_p_ζ, synapse = None
        - ė_ζ → u_ζ, transform = k_d_ζ, synapse = None
        - for ζ ∈ {x, y, z} do:
            - ∫ e_ζ → u_ζ, transform = 1, synapse = None
          end for
          
    ■ Subsystem coupling:
        - Connect the coupled states into G'_ζ and compute the input υ_ζ
        - E.g., F'_θ[0] → G'_x, synapse = None
        - Call function 'upsilon_x' to compute υ_x = -g * tan(θ)
End While
```

---

## 4. Simulation

### 4.1. Simulink Simulation
The implementation of the classic control architecture was developed in **MATLAB/Simulink R2025a**. For this benchmark, the rotor-level allocation is replaced by the virtual total thrust and body-axis command inputs, so that the comparison focuses on the closed-loop controller and its neural realization. 

The cascaded baseline comprises outer position loops for $\theta_d$ and $\phi_d$ generation, inner attitude loops for $u_\theta$, $u_\phi$, and $u_\psi$, and the control law.

### 4.2. Nengo Simulation
The control architecture described in the Neural Controller section was implemented in NEF with a postsynaptic time constant of 0.1s, and 200 neurons per ensemble and with an independent process to MATLAB/Simulink implementation.

The controller gains and simulation parameters are detailed in the table below:

| Controller | Controlled DOF | $k_p$ (SNN) | $k_d$ (SNN) | $k_i$ (SNN) | $k_p$ (Classical) | $k_d$ (Classical) | $k_i$ (Classical) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **PID$_x$** | Position $x_\mathcal{I}$ | 8.8 | 0.1 | 0.3 | 1.6 | 5.0 | 0.08 |
| **PID$_y$** | Position $y_\mathcal{I}$ | 10.3 | 4.2 | 0.7 | 3.0 | 2.0 | 0.10 |
| **PID$_z$** | Position $z_\mathcal{I}$ | 40 | 5.0 | 0.1 | 40 | 5.0 | 2.00 |
| **PD$_\theta$** | Attitude pitch $\theta$ | 90 | 12.1 | -- | 120 | 8.0 | -- |
| **PD$_\phi$** | Attitude roll $\phi$ | 90 | 12.0 | -- | 40 | 8.0 | -- |
| **PD$_\psi$** | Attitude yaw $\psi$ | 90 | 70 | -- | 7.0 | 4.0 | -- |

### General Parameters
* **Integration method:** NEF recurrent dynamics (SNN/Nengo) vs. Fourth-order Runge-Kutta (MATLAB/Simulink)
* **Vehicle mass ($m$):** $1\text{ kg}$
* **Neurons per ensemble ($J$):** 200 (SNN) / -- (Classical)
* **Postsynaptic time constant ($\tau$):** $0.1\text{ s}$ (SNN) / -- (Classical)

---

## 📈 Simulation Results

The experiments compare the translational and attitude responses of the SNN-based controller against the classical MATLAB/Simulink implementation and reference trajectories.

Both controllers exhibit similar trajectory-tracking performance, with the neural controller achieving high agreement with its classical counterpart across all degrees of freedom.

### 1. Closed-Loop States
Comparison of translational positions and attitude angles.
* **Reference:** Dotted black (`···`)
* **MATLAB/Simulink:** Solid blue (`—`)
* **SNN/Nengo:** Solid red (`—`)

[Closed-loop_states.pdf](https://github.com/user-attachments/files/32490202/fig01_states_v5_rad.pdf)

<img width="925" height="1124" alt="fig01_states_v5_rad_page-0001" src="https://github.com/user-attachments/assets/7ab596bb-09cb-4d4f-909c-33d5d97584ce" />



### 2. Control Commands
Total thrust $u$, virtual translational commands $u_x$ and $u_y$, and normalized attitude commands $u_\psi$, $u_\theta$, and $u_\phi$.

<img width="950" height="1124" alt="fig02_commands_v5_rads2_page-0001" src="https://github.com/user-attachments/assets/351d4ee5-9e15-4b17-be22-a77a5d7a1cb3" />


[Control_commands.pdf](https://github.com/user-attachments/files/32490205/fig02_commands_v5_rads2.pdf)


---

## 📐 Quantitative Performance Evaluation

The quantitative comparison between the two control strategies was conducted using error performance indices:

1. **RMSE** (Root Mean Square Error)
2. **ISE** (Integral of Squared Error)
3. **Correlation coefficient ($\rho_{\mathrm{SNN,cls}}$):** Quantifies the structural similarity between the SNN-based and classical controller signals.

> **Note:** While $\rho_{\mathrm{SNN,cls}}$ evaluates the structural similarity between the SNN and classical outputs, RMSE and ISE measure the tracking error with respect to the desired reference trajectory.
>
> Run Hexa_code_completly.py in path: Nengo/Integracion_3subsistemas. After run plots to generate position, orientacion and inputs’ plots
