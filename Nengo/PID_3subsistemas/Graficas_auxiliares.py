import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from mpl_toolkits.mplot3d import Axes3D #axes3d

import math
import matplotlib.ticker as ticker
from matplotlib.ticker import ScalarFormatter, FormatStrFormatter
plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
plt.rcParams['font.serif'] = 'DejaVu Serif'

# Leer el archivo CSV y cargarlo en un DataFrame
datosX = pd.read_csv('simulink_data_x.csv', header=None)
datosTheta = pd.read_csv('simulink_data_theta.csv', header=None)

datosY = pd.read_csv('simulink_data_y.csv', header=None)
datosPhi = pd.read_csv('simulink_data_phi.csv', header=None)


datosZ = pd.read_csv('simulink_data_z.csv', header=None)



num_xticks=5
num_yticks=5

Grosor_linea = 3

# Function to convert string representation to float
def str_to_float(string):
    # Remove square brackets and split by spaces
    values = string.replace('[', '').replace(']', '').split()
    # Convert each part to float and return as numpy array
    return np.array([float(value) for value in values])

# Crear una variable de índice basada en la longitud de los datos

Array_without_str = datosX.applymap(str_to_float).values.flatten()
#Array_without_str_ref = datosTheta.applymap(str_to_float).values.flatten()

Tiempo = [arr[0] for arr in Array_without_str]###Extraer valores
XValores = [arr[1] for arr in Array_without_str] ###Extraer valores
XDotValores = [arr[2] for arr in Array_without_str] ###Extraer valores
##############################################################################################################
##############################################################################################################
Array_without_str = datosY.applymap(str_to_float).values.flatten()
#Array_without_str_ref = datosTheta.applymap(str_to_float).values.flatten()

YValores = [arr[1] for arr in Array_without_str] ###Extraer valores
YDotValores = [arr[2] for arr in Array_without_str] ###Extraer valores
##############################################################################################################
##############################################################################################################
Array_without_str = datosZ.applymap(str_to_float).values.flatten()
#Array_without_str_ref = datosTheta.applymap(str_to_float).values.flatten()

ZValores = [arr[1] for arr in Array_without_str] ###Extraer valores
ZDotValores = [arr[2] for arr in Array_without_str] ###Extraer valores
##############################################################################################################
##############################################################################################################
Array_without_str = datosTheta.applymap(str_to_float).values.flatten()
#Array_without_str_ref = datosTheta.applymap(str_to_float).values.flatten()

ThetaValores = [arr[1] for arr in Array_without_str] ###Extraer valores
ThetaDotValores = [arr[2] for arr in Array_without_str] ###Extraer valores
##############################################################################################################
##############################################################################################################
Array_without_str = datosPhi.applymap(str_to_float).values.flatten()
#Array_without_str_ref = datosTheta.applymap(str_to_float).values.flatten()

PhiValores = [arr[1] for arr in Array_without_str] ###Extraer valores
PhiDotValores = [arr[2] for arr in Array_without_str] ###Extraer valores







# ValorX = datosX.applymap(str_to_float).values.flatten()
# ValorY = datosY.applymap(str_to_float).values.flatten()
# ValorZ = datosZ.applymap(str_to_float).values.flatten()

# ValorTheta = datosTheta.applymap(str_to_float).values.flatten()
# ValorPhi = datosPhi.applymap(str_to_float).values.flatten()

# u1 = InputX.values.flatten()
# u2 = InputY.values.flatten()
# u3 = InputZ.values.flatten()


# ErrorZ = RefZ - ZValores
# ErrorX = RefX - XValores
# ErrorY = RefY - YValores



dt = 0.001



#breakpoint()

# x = np.arange(len(datosX.columns))
# # # Definir el número de elementos
# num_elementos = len(x)

# # Definir la frecuencia de muestreo inicial (30 Hz)
# fs_inicial = 400  # Hz
# # Duración total en segundos
# duration_seconds = num_elementos / fs_inicial

# time_vector = np.linspace(0, duration_seconds, num_elementos)

# # Combinar los vectores de tiempo
# tiempo= time_vector#np.concatenate((tiempo_30s, tiempo_restante))


######################################################################################################################################
######################################### GRAFICAS COMBINADAS ########################################################################
fig, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(12, 8))

# Grafica Z
ax4.plot(Tiempo, ZValores, linewidth=Grosor_linea, label=r'$z$')
###ax4.plot(tiempo, RefZ, "--", linewidth=Grosor_linea, label=r'$z_{ref}$', color='red')
#ax4.set_xlabel('Time [s]', fontsize=28)
#ax4.set_ylabel('Distance [m]', fontsize=28)
ax4.grid(True, linestyle='-.')
ax4.legend(fontsize='28') 
ax4.tick_params(axis='both', labelsize=28)

# Grafica X
ax5.plot(Tiempo, XValores, linewidth=Grosor_linea, label=r'$x$')
###ax5.plot(tiempo, RefX, "--", linewidth=Grosor_linea, label=r'$x_{ref}$', color='red')
#ax5.set_xlabel('Time [s]', fontsize=28)
ax5.set_ylabel('Distance [m]', fontsize=28)
ax5.grid(True, linestyle='-.')
ax5.legend(loc='upper left', fontsize='28')
ax5.tick_params(axis='both', labelsize=28)

# Grafica Y
ax6.plot(Tiempo, YValores, linewidth=Grosor_linea, label=r'$y$')
####ax6.plot(tiempo, RefY, "--", linewidth=Grosor_linea, label=r'$y_{ref}$', color='red')
ax6.set_xlabel('Time [s]', fontsize=28)
#ax6.set_ylabel('Distance [m]', fontsize=28)
ax6.grid(True, linestyle='-.')
ax6.legend(loc='upper left', fontsize='28')
ax6.tick_params(axis='both', labelsize=28)

plt.tight_layout()
plt.savefig("Traslacionales.pdf")

# #################################################ERRORES##########################################################
# ###############################################GRAFICAS###########################################################
# # Crear una figura con tres subgráficas
# fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))

# # Graficar el error de seguimiento para el eje z en la primera subgráfica
# ax1.plot(tiempo, ErrorZ, linewidth=Grosor_linea, label=r'$\epsilon_{z}$')
# #ax1.set_xlabel('Time[s]', fontsize=20)
# ax1.set_ylabel(r'$\epsilon_{z}$ [m]', fontsize=28)
# ax1.grid(True, linestyle='-.')
# ax1.legend(fontsize='28')
# ax1.tick_params(axis='x', labelsize=28)  # Tamaño de la fuente para los números del eje x
# ax1.tick_params(axis='y', labelsize=28)  # Tamaño de la fuente para los números del eje y

# # Graficar el error de seguimiento para el eje x en la segunda subgráfica
# ax2.plot(tiempo, ErrorX, linewidth=Grosor_linea, label=r'$\epsilon_{x}$')
# #ax2.set_xlabel('Time[s]', fontsize=20)
# ax2.set_ylabel(r'$\epsilon_{x}$ [m]', fontsize=28)
# ax2.grid(True, linestyle='-.')
# ax2.legend(fontsize='28')
# ax2.tick_params(axis='x', labelsize=28)  # Tamaño de la fuente para los números del eje x
# ax2.tick_params(axis='y', labelsize=28)  # Tamaño de la fuente para los números del eje y

# # Graficar el error de seguimiento para el eje y en la tercera subgráfica
# ax3.plot(tiempo, ErrorY, linewidth=Grosor_linea, label=r'$\epsilon_{y}$')
# ax3.set_xlabel('Time[s]', fontsize=28)
# ax3.set_ylabel(r'$\epsilon_{y}$ [m]', fontsize=28)
# ax3.grid(True, linestyle='-.')
# ax3.legend(fontsize='28')
# ax3.tick_params(axis='x', labelsize=28)  # Tamaño de la fuente para los números del eje x
# ax3.tick_params(axis='y', labelsize=28)  # Tamaño de la fuente para los números del eje y

# # Ajustar el diseño de las subgráficas para evitar superposiciones
# plt.tight_layout()

# # Guardar la figura en un archivo PDF
# plt.savefig("ErrorSeguimiento.pdf")


###############################################################################################################
###############################################################################################################


fig, (ax7, ax8) = plt.subplots(2, 1, figsize=(12, 8))

# Grafica Theta
ax7.plot(Tiempo, ThetaValores, linewidth=Grosor_linea, label=r'$z$')
#ax4.set_xlabel('Time [s]', fontsize=28)
ax7.set_ylabel('u1', fontsize=28)
ax7.grid(True, linestyle='-.')
ax7.legend(fontsize='28') 
ax7.tick_params(axis='both', labelsize=28)

# Grafica Phi
ax8.plot(Tiempo, PhiValores, linewidth=Grosor_linea, label=r'$x$')
#ax5.set_xlabel('Time [s]', fontsize=28)
ax8.set_ylabel('u2', fontsize=28)
ax8.grid(True, linestyle='-.')
ax8.legend(loc='upper left', fontsize='28')
ax8.tick_params(axis='both', labelsize=28)



plt.tight_layout()
plt.savefig("Rotacionales.pdf")


################################################################################################################

# fig = plt.figure()
# ax8 =  fig.add_subplot(projection='3d')

# Positions2 = np.array(ZValores).reshape(-1, 1)
# Positions1 = np.array(ReferenciaZ).reshape(-1, 1)

# ax8.plot(XValores, YValores, ZValores,linewidth=Grosor_linea, label='UAS trajectory')
# ax8.plot(RefX, RefY, RefZ,"--",linewidth=Grosor_linea, label='Desired trajectory', color='red')
# ax8.legend(loc='upper left', fontsize=14)

# # Configurar los límites de los ejes
# # Asegúrate de que los valores en los ejes x, y, z sean los mismos para que la misma cantidad de elementos aparezca en cada eje
# valores_min = min(min(XValores), min(YValores), min(RefX), min(RefY))
# valores_max = max(max(XValores), max(YValores), max(RefX), max(RefY))

# # Establecer los límites de los ejes x, y, z
# # Set the number of ticks on the x-axis and y-axis
# ax8.locator_params(axis='x', nbins=5)
# ax8.locator_params(axis='y', nbins=5)
# ax8.locator_params(axis='z', nbins=5)

# ax8.set_xlim(valores_min, valores_max)
# ax8.set_ylim(valores_min, valores_max)
# #ax8.set_zlim(valores_min, valores_max)


# # Etiquetas de los ejes
# ax8.set_xlabel(r'$x$[m]', fontsize=14)
# ax8.set_ylabel(r'$y$[m]', fontsize=14, rotation=60)
# ax8.set_zlabel(r'$z$[m]', fontsize=14,rotation=60)
# # Para el eje x
# ax8.tick_params(axis='x', pad=2)  # Ajusta el espacio entre los números y las etiquetas del eje x

# # Para el eje y
# ax8.tick_params(axis='y', pad=2)  # Ajusta el espacio entre los números y las etiquetas del eje y

# # Para el eje z (en una gráfica 3D)
# ax8.tick_params(axis='z', pad=2)  # Ajusta el espacio entre los números y las etiquetas del eje z

# #ax8.yaxis._axinfo['label']['space_factor'] = 10.0

# plt.xticks(fontsize='14')
# plt.yticks(fontsize='14')
# ax8.tick_params(axis='z', labelsize=14)  # Tamaño de la fuente para los ticks del eje z


# plt.savefig("MovimientoUAV.pdf")
