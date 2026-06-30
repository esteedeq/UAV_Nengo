import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from mpl_toolkits.mplot3d import Axes3D
import math
import matplotlib.ticker as ticker
from matplotlib.ticker import ScalarFormatter, FormatStrFormatter

# Configuración de fuentes
plt.rcParams['font.family'] = 'serif'
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

# Configuración de gráficos
num_xticks = 5
num_yticks = 5
Grosor_linea = 3

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

# # Ejemplo de cómo graficar los datos (opcional)
# plt.figure(figsize=(12, 8))

# # Graficar posiciones
# plt.subplot(2, 2, 1)
# plt.plot(tiempo_X, XValores, linewidth=Grosor_linea, label='X')
# plt.plot(tiempo_Y, YValores, linewidth=Grosor_linea, label='Y')
# plt.plot(tiempo_Z, ZValores, linewidth=Grosor_linea, label='Z')
# plt.title('Posiciones')
# plt.xlabel('Tiempo (s)')
# plt.ylabel('Posición')
# plt.legend()
# plt.grid(True)

# # Graficar ángulos
# plt.subplot(2, 2, 2)
# plt.plot(tiempo_Theta, ThetaValores, linewidth=Grosor_linea, label='Theta')
# plt.plot(tiempo_Phi, PhiValores, linewidth=Grosor_linea, label='Phi')
# plt.title('Ángulos')
# plt.xlabel('Tiempo (s)')
# plt.ylabel('Ángulo (rad)')
# plt.legend()
# plt.grid(True)

# # Graficar velocidades lineales
# plt.subplot(2, 2, 3)
# plt.plot(tiempo_X, XDotValores, linewidth=Grosor_linea, label='X_dot')
# plt.plot(tiempo_Y, YDotValores, linewidth=Grosor_linea, label='Y_dot')
# plt.plot(tiempo_Z, ZDotValores, linewidth=Grosor_linea, label='Z_dot')
# plt.title('Velocidades Lineales')
# plt.xlabel('Tiempo (s)')
# plt.ylabel('Velocidad')
# plt.legend()
# plt.grid(True)

# # Graficar velocidades angulares
# plt.subplot(2, 2, 4)
# plt.plot(tiempo_Theta, ThetaDotValores, linewidth=Grosor_linea, label='Theta_dot')
# plt.plot(tiempo_Phi, PhiDotValores, linewidth=Grosor_linea, label='Phi_dot')
# plt.title('Velocidades Angulares')
# plt.xlabel('Tiempo (s)')
# plt.ylabel('Velocidad Angular')
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.show()

# # También puedes acceder a los datos como arrays de numpy
# print(f"Datos X: {len(XValores)} puntos")
# print(f"Datos Y: {len(YValores)} puntos")
# print(f"Datos Z: {len(ZValores)} puntos")
# print(f"Datos Theta: {len(ThetaValores)} puntos")
# print(f"Datos Phi: {len(PhiValores)} puntos")