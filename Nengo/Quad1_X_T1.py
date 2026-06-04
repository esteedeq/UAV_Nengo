import nengo
import numpy as np
import matplotlib.pyplot as plt
#Constantes fisicas
m=1
g=9.81
 #Funciones trigonometricas
tan=np.tan
sin=np.sin
cos=np.cos
arctan2=np.arctan2
pi=np.pi
#Modelo
t_syn=0.1
E_syn=0.05
model=nengo.Network(seed=42, label="Quadrirotor_Z_X_control")
#Saturacion de neuronas
Sx=5
St=pi*40/180
Sx=5
Sz=15
#Ganancias
kpx=2.3
kdx=2
kpt=70 #Tenia 120
kdt=11 #Tenia 8
kpz=40
kdz=5
with model:
   "Control de Posicion x"
   #Representacion de la forma f' y g' de las matrices en espacio de estados  de x
   Fx=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sx)
   Gx=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
   
   def fx_fun(x):
       return [x[1]*t_syn,0]+x 
   def gx_fun(x):
       return [0,x[0]*t_syn]
   nengo.Connection(Gx,Fx,synapse=t_syn,function=gx_fun)
   nengo.Connection(Fx,Fx,synapse=t_syn,function=fx_fun)
   
   #Control para obtener a theta_d 
    #Referencia
   ref_x=nengo.Node(lambda x, k=1, centro=5: 1 / (1 + np.exp(-k * (x - centro))))
   En_refx=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx,neuron_type=nengo.Direct())
   nengo.Connection(ref_x,En_refx)
   
   #Errores en x 
   Err_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
   nengo.Connection(Fx[0],Err_x,synapse=E_syn)
   nengo.Connection(En_refx,Err_x,transform=-1,synapse=E_syn)
   #Derivada del error en  x 
   D_err_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
   nengo.Connection(Err_x,D_err_x,synapse=E_syn,transform=20)
   nengo.Connection(Err_x,D_err_x,synapse=t_syn,transform=-20)

   #Control, para la obtencion de u_x=g*tan(theta_d)
   U_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx*2)
   nengo.Connection(Err_x,U_x,transform=-kpx)
   nengo.Connection(D_err_x,U_x,transform=-kdx) 
   #### AQUI u = g*tan(theta_d) + (error*Kpt + D_error*Kdt)
   #### u = g*theta_d + (error*Kpt + D_error*Kdt)
   #tan(theta) = theta si el valor de theta es pequeño

   
#    nengo.Connection(U_x,Gx,function=None,synapse=None)


   "Control de theta"
   At=nengo.Ensemble(n_neurons=500,dimensions=2,radius=St)
   Bt=nengo.Ensemble(n_neurons=200,dimensions=1,radius=St)
   
   def A_fun(x):
       return [x[1]*t_syn,0]+x
   def B_fun(x):
       return [0, x[0]*t_syn]
   nengo.Connection(Bt,At,function=B_fun,synapse=t_syn)
   nengo.Connection(At,At,function=A_fun,synapse=t_syn)
   
   #Referencia
   
   def theta_d(x): 
       return arctan2(x,g)
   ref_t=nengo.Ensemble(n_neurons=200, dimensions=1,radius=St)
   nengo.Connection(U_x,ref_t,function=theta_d,synapse=t_syn)
   En_ref_t=nengo.Ensemble(n_neurons=200, dimensions=1,radius=St,neuron_type=nengo.Direct())
   nengo.Connection(ref_t,En_ref_t,synapse=t_syn)
   
   #Error en theta
   Err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.05)
   nengo.Connection(At[0],Err_t,synapse=E_syn)
   nengo.Connection(En_ref_t,Err_t,synapse=E_syn,transform=-1)

   #Derivada del error
   d_ref_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=.4)
   nengo.Connection(En_ref_t,d_ref_t,synapse=t_syn,transform=20)
   nengo.Connection(En_ref_t,d_ref_t,synapse=E_syn,transform=-20)

   D_err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=St)
   nengo.Connection(At[1],D_err_t,synapse=E_syn)
   nengo.Connection(d_ref_t,D_err_t,synapse=E_syn,transform=-1)
    
   #Control de theta
   U_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1.3)
   nengo.Connection(Err_t,U_t,transform=-kpt)
   nengo.Connection(D_err_t,U_t,transform=-kdt)
    
   #Conexion con B
   nengo.Connection(U_t,Bt,function=None,transform=None)    


   #Coneccion final de u_x a B en la dinamica de X
   nengo.Connection(At[0],U_x,transform=-g)
   
   nengo.Connection(U_x,Gx,function=None,synapse=None)
       
   
   #Simulacion
   Fx_p=nengo.Probe(Fx,synapse=t_syn)
   Rx_p=nengo.Probe(En_refx,synapse=t_syn)
   Erx_p=nengo.Probe(Err_x,synapse=t_syn)
   Ux_p=nengo.Probe(U_x,synapse=t_syn)
    
   At_p=nengo.Probe(At,synapse=t_syn)
   Et_p=nengo.Probe(Err_t,synapse=t_syn)
   DEt_p=nengo.Probe(D_err_t,synapse=t_syn)
   Ut_p=nengo.Probe(U_t,synapse=t_syn)
   Rt_p=nengo.Probe(En_ref_t,synapse=t_syn)
   rt_p=nengo.Probe(ref_t,synapse=t_syn)

   with nengo.Simulator(model) as sim:
       sim.run(20)
   t = sim.trange()
    
   #Calculo de maximos para la saturacion de los ensambles
   Erx_data=sim.data[Erx_p]
   Ux_data=sim.data[Ux_p]
    

   m_Er=max(Erx_data)
   m_U=max(Ux_data)

   
   print("El error maximo en x es :",m_Er)
   print("La entrada maxima en U_x es:",m_U)
    
   Ertheta_data=sim.data[Et_p]
   DErtheta_data=sim.data[DEt_p]
   Utheta_data=sim.data[Ut_p]
    #Datos en phi
   m_Ert=max(Ertheta_data)
   m_dErt=max(DErtheta_data)
   m_Ut=max(Utheta_data)
   print("El error maximo en theta es :",m_Ert)
   print("La derivada del error maxima en theta es:", m_dErt)
   print("La entrada maxima en U_t es:",m_Ut)
   
   #Exportacion de datos a csv 
   data = sim.data[Fx_p]
   export_data = np.column_stack((t, data))
   np.savetxt('simulink_data_x.csv', 
              export_data, 
              delimiter=',', 
              header='Time,Position', 
              comments='')
   print("Datos exportados a simulink_data_x.csv")
   #Exportacion de datos a csv 
   data = sim.data[At_p]
   export_data = np.column_stack((t, data))
   np.savetxt('simulink_data_theta.csv', 
              export_data, 
              delimiter=',', 
              header='Time,Position', 
              comments='')
   print("Datos exportados a simulink_data_Theta.csv")
   
   
   def plot_X(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
       
       plt.figure(figsize=(9, 10))
       plt.plot(t, data[Fx_p][:,0], label="Position X", color='red')
       plt.plot(t, data[Rx_p], label="Ref x", color='green')
       plt.plot(t, data[Erx_p], label="Err x ", color='blue')

   
       plt.xlabel("Time [s]", fontsize=32)
       plt.ylabel("Position x [m]", fontsize=32)
       plt.grid(True)
       plt.legend(fontsize=32) 
   
       # Set the number of ticks on the x-axis and y-axis
       plt.locator_params(axis='x', nbins=num_xticks)
       plt.locator_params(axis='y', nbins=num_yticks)
   
       # Set the x and y axis limits if provided
       if xmin is not None and xmax is not None:
           plt.xlim(xmin, xmax)
       if ymin is not None and ymax is not None:
           plt.ylim(ymin, ymax)
   
       plt.xticks(fontsize=28)
       plt.yticks(fontsize=28)
       
       #file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position X'
       plt.savefig('PositionX', format="pdf")   
   
   plot_X(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=20, ymin=-5, ymax=5)
   
   def plot_t(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[At_p][:,0], label="Position theta", color='red')
        plt.plot(t, data[Rt_p], label="Ref theta", color='green')
        plt.plot(t, data[Et_p], label="Err theta ", color='blue')

        plt.xlabel("Time [s]", fontsize=32)
        plt.ylabel("Position theta [rad]", fontsize=32)
        plt.grid(True)
        plt.legend(fontsize=32) 
    
        # Set the number of ticks on the x-axis and y-axis
        plt.locator_params(axis='x', nbins=num_xticks)
        plt.locator_params(axis='y', nbins=num_yticks)
    
        # Set the x and y axis limits if provided
        if xmin is not None and xmax is not None:
            plt.xlim(xmin, xmax)
        if ymin is not None and ymax is not None:
            plt.ylim(ymin, ymax)
    
        plt.xticks(fontsize=28)
        plt.yticks(fontsize=28)
        
        #file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Theta'
        plt.savefig('Position Theta', format="pdf")   
        plt.show()
    
   plot_t(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=20, ymin=-pi*10/180, ymax=pi*10/180)
   
  