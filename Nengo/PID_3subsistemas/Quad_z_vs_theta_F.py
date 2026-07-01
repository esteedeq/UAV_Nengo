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
model=nengo.Network(label="Quadrirotor_Z_X_control")
#Saturacion de neuronas
Sx=5
St=pi*40/180
Sx=5
Sz=15
#Ganancias
kpx=1.6
kdx=2
kpt=120
kdt=8
kpz=40;
kdz=5;

phi=0
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
   
   nengo.Connection(U_x,Gx,function=None,synapse=None)


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
   Err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.03)
   nengo.Connection(At[0],Err_t,synapse=E_syn)
   nengo.Connection(En_ref_t,Err_t,synapse=E_syn,transform=-1)

   #Derivada del error   
   d_ref_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=.3)
   nengo.Connection(En_ref_t,d_ref_t,synapse=t_syn,transform=20)
   nengo.Connection(En_ref_t,d_ref_t,synapse=E_syn,transform=-20)

   D_err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=St)
   nengo.Connection(At[1],D_err_t,synapse=E_syn)
   nengo.Connection(d_ref_t,D_err_t,synapse=E_syn,transform=-1)
    
   #Control de theta
   U_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1.5)
   nengo.Connection(Err_t,U_t,transform=-kpt)
   nengo.Connection(D_err_t,U_t,transform=-kdt)
    
   #Conexion con B
   nengo.Connection(U_t,Bt,function=None,transform=None)    
     
   "Control de Posicion z"
   # theta_r=nengo.Node(lambda x: 5*pi/180*cos(x))
   CC=nengo.Ensemble(n_neurons=200,dimensions=2,neuron_type=nengo.Direct())
   nengo.Connection(At[0],CC[1])
   #Representacion de la forma f' y g' de las matrices en espacio de estados  de x
   Fz=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sz)
   Gz=nengo.Ensemble(n_neurons=200,dimensions=1,radius=28.5)
   G=nengo.Node(g)
   def fz_fun(x):
       return [x[1]*t_syn,0]+x 
   def gz_fun(x):
       return [0,((x[0]*cos(x[1])*cos(phi))/m -g)*t_syn]
   nengo.Connection(Gz,CC[0])
   nengo.Connection(CC,Fz,synapse=t_syn,function=gz_fun)
   nengo.Connection(Fz,Fz,synapse=t_syn,function=fz_fun)
   
    #Referencia
   ref_z=nengo.Node(lambda x, k=1, centro=4: 3/ (1 + np.exp(-k * (x - centro))))
   En_refz=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sz,neuron_type=nengo.Direct())
   nengo.Connection(ref_z,En_refz)
   
   #Errores en z
   Err_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=0.3)
   nengo.Connection(Fz[0],Err_z,synapse=E_syn)
   nengo.Connection(En_refz,Err_z,transform=-1,synapse=E_syn)
   #Derivada de z deseado
   d_ref_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=1)
   nengo.Connection(En_refz,d_ref_z,synapse=E_syn,transform=-20)
   nengo.Connection(En_refz,d_ref_z,synapse=t_syn,transform=20)
   
   #Derivadas del error en z
   D_Err_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=6.5)
   nengo.Connection(Fz[1],D_Err_z,synapse=E_syn)
   nengo.Connection(d_ref_z,D_Err_z,transform=-1,synapse=E_syn)
   
   #Control, para la obtencion de u
   def u_fun(x): 
       return x[0]/(cos(x[1])*cos(phi))
  
   U=nengo.Ensemble(n_neurons=100,dimensions=1,radius=28.5)
   nengo.Connection(G,U,transform=m)
   nengo.Connection(Err_z,U,transform=-kpz)
   nengo.Connection(D_Err_z,U,transform=-kdz)
   CC2=nengo.Ensemble(n_neurons=200,dimensions=2,neuron_type=nengo.Direct())
   nengo.Connection(U,CC2[0])
   nengo.Connection(At[0],CC2[1])
   nengo.Connection(CC2,Gz,function=u_fun,synapse=None)
   
   #Simulacion x theta 
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
   
   #Simulacion z
   Fz_p=nengo.Probe(Fz,synapse=t_syn)
   Rz_p=nengo.Probe(En_refz,synapse=t_syn)
   Erz_p=nengo.Probe(Err_z,synapse=t_syn)
   DErr_z=nengo.Probe(D_Err_z,synapse=t_syn)
   U_p=nengo.Probe(U,synapse=t_syn)
   Gz_p=nengo.Probe(Gz,synapse=t_syn)

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
   # data = sim.data[Fx_p]
   # export_data = np.column_stack((t, data))
   # np.savetxt('/Users/Usuario/Documents/IA/ESTANCIA/Quad/simulink_data_x.csv', 
   #            export_data, 
   #            delimiter=',', 
   #            header='Time,Position', 
   #            comments='')
   print("Datos exportados a simulink_data_x.csv")
   #Exportacion de datos a csv 
   # data = sim.data[At_p]
   # export_data = np.column_stack((t, data))
   # np.savetxt('/Users/Usuario/Documents/IA/ESTANCIA/Quad/simulink_data_theta.csv', 
   #            export_data, 
   #            delimiter=',', 
   #            header='Time,Position', 
   #            comments='')
   print("Datos exportados a simulink_data_Theta.csv")
   Er_data=sim.data[Erz_p]
   DEr_data=sim.data[DErr_z]
   U_data=sim.data[U_p]
   m_Er=max(Er_data)
   m_dEr=max(DEr_data)
   m_U=max(U_data)
   print("El error maximo es :",m_Er)
   print("La derivada del error maxima es:", m_dEr)
   print("El empuje maximo es:",m_U)
   #Exportacion de datos a csv 
   # data = sim.data[Fz_p]
   # export_data = np.column_stack((t, data))
   # np.savetxt('/Users/Usuario/Documents/IA/ESTANCIA/Quad/simulink_data_z.csv', 
   #            export_data, 
   #            delimiter=',', 
   #            header='Time,Position', 
   #            comments='')
   print("Datos exportados a simulink_data.csv")
   
   
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
       
       # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position X'
       # plt.savefig(file, format="pdf", bbox_inches='tight')   
       
       
   
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
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Theta'
        # plt.savefig(file, format="pdf", bbox_inches='tight')   
    
   plot_t(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=20, ymin=-pi*50/180, ymax=pi*50/180)
   
   def plot_Z(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
       
       plt.figure(figsize=(9, 10))
       plt.plot(t, data[Fz_p][:,0], label="Position z", color='red')
       plt.plot(t, data[Rz_p], label="Ref z", color='green')
       plt.plot(t, data[Erz_p], label="Err z ", color='blue')