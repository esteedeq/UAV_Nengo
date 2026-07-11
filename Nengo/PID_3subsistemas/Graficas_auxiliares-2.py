import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
#from sklearn.preprocessing import MinMaxScaler
from mpl_toolkits.mplot3d import Axes3D #axes3d
import math
import matplotlib.ticker as ticker
from matplotlib.ticker import ScalarFormatter, FormatStrFormatter
from matplotlib import rc

rc('text', usetex=True)
rc('font', family='serif')

import matplotlib.ticker as ticker
from matplotlib.ticker import ScalarFormatter, FormatStrFormatter
plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
plt.rcParams['font.serif'] = 'DejaVu Serif'

# Leer los archivos CSV correctamente
# Tus archivos tienen header con 'Time,Position' y 3 columnas: tiempo, posición, velocidad
datosX = pd.read_csv('simulink_data_x.csv', skiprows=1, header=None)  # Saltar el header
datosTheta = pd.read_csv('simulink_data_theta.csv', skiprows=1, header=None)
datosY = pd.read_csv('simulink_data_y.csv', skiprows=1, header=None)
datosPhi = pd.read_csv('simulink_data_phi.csv', skiprows=1, header=None)
datosZ = pd.read_csv('simulink_data_z.csv', skiprows=1, header=None)

# Extraer los datos directamente (sin necesidad de str_to_float)
# Las columnas son: [0] = tiempo, [1] = posición, [2] = velocidad

# Tus archivos tienen header con 'Time,Position' y 3 columnas: tiempo, posición, velocidad
datosInputZ = pd.read_csv('simulink_data_Uz.csv', skiprows=1, header=None)  # Saltar el header
datosInputTheta = pd.read_csv('simulink_data_Tautheta.csv', skiprows=1, header=None)
datosInputPhi = pd.read_csv('simulink_data_TauPhi.csv', skiprows=1, header=None)

# Leer los archivos CSV correctamente
datosInputX = pd.read_csv('simulink_data_Ux.csv', skiprows=1, header=None)  # Saltar el header
datosInputY = pd.read_csv('simulink_data_Uy.csv', skiprows=1, header=None)




# Datos X
tiempo_X = datosX[0].values
XValores = datosX[1].values
XDotValores = datosX[2].values

# Datos Y
tiempo_Y = datosY[0].values
YValores = datosY[1].values
YDotValores = datosY[2].values

# Datos Z
tiempo_Z = datosZ[0].values
ZValores = datosZ[1].values
ZDotValores = datosZ[2].values

# Datos Theta
tiempo_Theta = datosTheta[0].values
ThetaValores = datosTheta[1].values
ThetaDotValores = datosTheta[2].values

# Datos Phi
tiempo_Phi = datosPhi[0].values
PhiValores = datosPhi[1].values
PhiDotValores = datosPhi[2].values

# Verificar que todos los tiempos son iguales (opcional)
# print("Tiempos iguales:", np.allclose(tiempo_X, tiempo_Y))
#####################################################################################################3
# Datos In_Z
tiempo_InputZ = datosInputZ[0].values
InputValoresZ = datosInputZ[1].values

# Datos In_Theta
tiempo_InputTheta = datosInputTheta[0].values
InputValoresTheta = datosInputTheta[1].values

# Datos In_Phi
tiempo_InputPhi = datosInputPhi[0].values
InputValoresPhi = datosInputPhi[1].values

#####################################################################################################3
# Datos In_X
tiempo_InputX = datosInputX[0].values
InputValoresX = datosInputX[1].values

# Datos In_y
tiempo_InputY = datosInputY[0].values
InputValoresY = datosInputY[1].values




# Configuración de gráficos
num_xticks = 5
num_yticks = 5
Grosor_linea = 2

# ============================================
# 1. GRÁFICAS DE POSICIONES TRASLACIONALES (X, Y, Z)
# ============================================
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

# Gráfica Z
ax1.plot(tiempo_Z, ZValores, linewidth=Grosor_linea, label=r'$z$')
#ax1.set_ylabel('Distance [m]', fontsize=28)
ax1.grid(True, linestyle='-.')
ax1.legend(fontsize='28', loc='upper right')
ax1.tick_params(axis='both', labelsize=28)

# Gráfica X
ax2.plot(tiempo_X, XValores, linewidth=Grosor_linea, label=r'$x$')
ax2.set_ylabel('Distance [m]', fontsize=28)
ax2.grid(True, linestyle='-.')
ax2.legend(fontsize='28', loc='upper right')
ax2.tick_params(axis='both', labelsize=28)

# Gráfica Y
ax3.plot(tiempo_Y, YValores, linewidth=Grosor_linea, label=r'$y$')
ax3.set_xlabel('Time [s]', fontsize=28)
#ax3.set_ylabel('Distance [m]', fontsize=28)
ax3.grid(True, linestyle='-.')
ax3.legend(fontsize='28', loc='upper right')
ax3.tick_params(axis='both', labelsize=28)

plt.tight_layout()
plt.savefig("Posiciones_Traslacionales.pdf", bbox_inches='tight')
plt.close()

# ============================================
# 2. GRÁFICAS DE ÁNGULOS (Theta y Phi)
# ============================================
fig2, (ax4, ax5) = plt.subplots(2, 1, figsize=(12, 8))

# Gráfica Theta
ax4.plot(tiempo_Theta, ThetaValores, linewidth=Grosor_linea, label=r'$\theta$')
ax4.set_ylabel('Angle [rad]', fontsize=28)
ax4.grid(True, linestyle='-.')
ax4.legend(fontsize='28', loc='upper right')
ax4.tick_params(axis='both', labelsize=28)

# Gráfica Phi
ax5.plot(tiempo_Phi, PhiValores, linewidth=Grosor_linea, label=r'$\phi$')
ax5.set_xlabel('Time [s]', fontsize=28)
ax5.set_ylabel('Angle [rad]', fontsize=28)
ax5.grid(True, linestyle='-.')
ax5.legend(fontsize='28', loc='upper right')
ax5.tick_params(axis='both', labelsize=28)

plt.tight_layout()
plt.savefig("Angulos.pdf", bbox_inches='tight')
plt.close()


# ============================================
# 3. GRÁFICAS DE INPUTS (Uz, Tau_Theta y Tau_Phi)
# ============================================
fig3, (ax6, ax7, ax8) = plt.subplots(3, 1, figsize=(12, 8))

# Gráfica Theta
ax6.plot(tiempo_InputZ, InputValoresZ, linewidth=Grosor_linea, label=r'$U_z$')
ax6.set_ylabel('Trust[N]', fontsize=28)
ax6.grid(True, linestyle='-.')
ax6.legend(fontsize='28', loc='upper right')
ax6.tick_params(axis='both', labelsize=28)

# Gráfica Phi
ax7.plot(tiempo_InputTheta, InputValoresTheta, linewidth=Grosor_linea, label=r'$\tau_\theta$')
ax7.set_xlabel('Time [s]', fontsize=28)
ax7.set_ylabel('M [Nm]', fontsize=28)
ax7.grid(True, linestyle='-.')
ax7.legend(fontsize='28', loc='upper right')
ax7.tick_params(axis='both', labelsize=28)

# Gráfica Y
ax8.plot(tiempo_InputPhi, InputValoresPhi, linewidth=Grosor_linea, label=r'$\tau_\phi$')
ax8.set_xlabel('Time [s]', fontsize=28)
ax3.set_ylabel('M [Nm]', fontsize=28)
ax8.grid(True, linestyle='-.')
ax8.legend(fontsize='28', loc='upper right')
ax8.tick_params(axis='both', labelsize=28)

plt.tight_layout()
plt.savefig("Inputs.pdf", bbox_inches='tight')
plt.close()


# ============================================
# 4. GRÁFICAS DE INPUTS (Ux, Uy)
# ============================================
fig4, (ax4, ax5) = plt.subplots(2, 1, figsize=(12, 8))

# Gráfica Theta
ax4.plot(tiempo_InputX, InputValoresX, linewidth=Grosor_linea, label=r'$U_x$')
ax4.set_ylabel('Angle [rad]', fontsize=28)
ax4.grid(True, linestyle='-.')
ax4.legend(fontsize='28', loc='upper right')
ax4.tick_params(axis='both', labelsize=28)

# Gráfica Phi
ax5.plot(tiempo_InputY, InputValoresY, linewidth=Grosor_linea, label=r'$U_y$')
ax5.set_xlabel('Time [s]', fontsize=28)
ax5.set_ylabel('Angle [rad]', fontsize=28)
ax5.grid(True, linestyle='-.')
ax5.legend(fontsize='28', loc='upper right')
ax5.tick_params(axis='both', labelsize=28)

plt.tight_layout()
plt.savefig("InputsAuxiliar.pdf", bbox_inches='tight')
plt.close()

