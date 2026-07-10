import nengo
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D #axes3d
import plotly.graph_objects as go
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
model=nengo.Network(seed=40, label="Quadrirotor_Z_X_control")
#Saturacion de neuronas
Sx=5
St=pi*40/180
Sx=5
Sz=15
Sy=5
Sphi=pi*70/180 #pi*50/180
#Ganancias
kpx=8.8#2.2. 6.5, 5.8
kdx=0.1 #0.2
kix = 0.3 #0.2

kpt=90 #75.5
kdt=12.0 #20, 10

kpz=40
kdz=5
kiz=0.5

kpphi=90 #Tenia 130, 75
kdphi=12 #Tenia 55, 8


kpy=10 #Tenia 3, 30
kdy=4.2 #Tenia 2, 8
kiy = 0.7 #
#

#LIMITES EN LOS EJES DE LA GRAFICA
EjeX=60
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
    
        
    ref_x=nengo.Node(lambda x, k=1, centro=5: 1 / (1 + np.exp(-k * (x - centro))))
    #ref_x = nengo.Node(lambda t, k=1, centro=5: 0 if t < 5 else 1 / (1 + np.exp(-k * (t - centro))))
    En_refx=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx,neuron_type=nengo.Direct())
    nengo.Connection(ref_x,En_refx)
    
    #Errores en x 
    Err_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
    nengo.Connection(Fx[0],Err_x,synapse=E_syn, transform=-1)
    nengo.Connection(En_refx,Err_x,transform=1,synapse=E_syn)
    #Derivada del error en  x 
    D_err_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
    nengo.Connection(Err_x,D_err_x,synapse=E_syn,transform=20)
    nengo.Connection(Err_x,D_err_x,synapse=t_syn,transform=-20)

    #Integral del error en x
    In_Err_x = nengo.Ensemble(n_neurons=500, dimensions=1, radius=Sx)
    nengo.Connection(Err_x, In_Err_x, transform= kix, synapse=E_syn)
    nengo.Connection(In_Err_x, In_Err_x,transform=1, synapse=E_syn)
    

    #Control, para la obtencion de u_x=g*tan(theta_d)
    U_x=nengo.Ensemble(n_neurons=500,dimensions=1,radius=Sx*2)
    nengo.Connection(Err_x,U_x,transform=kpx)
    nengo.Connection(D_err_x,U_x,transform=kdx)
    nengo.Connection(In_Err_x,U_x,transform=1)

    #########nengo.Connection(U_x,Gx,function=None,synapse=None)


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
    Err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.1)
    nengo.Connection(At[0],Err_t,synapse=E_syn, transform=1)
    nengo.Connection(En_ref_t,Err_t,synapse=E_syn, transform=-1)

    #Derivada del error   
    d_ref_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1)
    nengo.Connection(En_ref_t,d_ref_t,synapse=t_syn,transform=20)
    nengo.Connection(En_ref_t,d_ref_t,synapse=E_syn,transform=-20)

    D_err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=St)
    nengo.Connection(At[1],D_err_t,synapse=E_syn)
    nengo.Connection(d_ref_t,D_err_t,synapse=E_syn,transform=-1)
     
    #Control de theta
    U_t=nengo.Ensemble(n_neurons=500,dimensions=1,radius=0.5)#Radius=1.5
    nengo.Connection(Err_t,U_t,transform=-kpt)
    nengo.Connection(D_err_t,U_t,transform=-kdt) 
     
    #Conexion con B
    nengo.Connection(U_t,Bt,function=None,transform=None)

    #Conecxion final de u_x a B en la dinamica de X
    def RetroTheta_X(x): 
        return -g*tan(x)
    nengo.Connection(At[0],U_x,function=RetroTheta_X) # -g tan(A[0])
    nengo.Connection(U_x,Gx,function=None,synapse=None)
    # Conecxion final de u_x a B en la dinamica de X
    # def upsilon_x(x): 
    #     return -g*tan(x)
    # nengo.Connection(At[0],Gx,function=upsilon_x,synapse=None)
    #################################################################################




    "Control de Posicion y"
    #Representacion de la forma f' y g' de las matrices en espacio de estados  de x
    Fy=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sy)
    Gy=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy)
    
    def fy_fun(x):
        return [x[1]*t_syn,0]+x 
    def gy_fun(x):
        return [0,x[0]*t_syn]
    nengo.Connection(Gy,Fy,synapse=t_syn,function=gy_fun)
    nengo.Connection(Fy,Fy,synapse=t_syn,function=fy_fun)


    #Control para obtener a phi_d 
     #Referencia
    # ref_y=nengo.Node(lambda x, k=1, centro=5:1.5 / (1 + np.exp(-k * (x - centro))))
    ref_y = nengo.Node(lambda t, k=1, centro=5: 0 if t < 15 else 1.5 / (1 + np.exp(-k * ((t-15) - centro))))
    En_refy=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy,neuron_type=nengo.Direct())
    nengo.Connection(ref_y,En_refy)
    
    #Errores en y
    Err_y=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy)
    nengo.Connection(Fy[0],Err_y,synapse=E_syn, transform=-1)
    nengo.Connection(En_refy,Err_y,transform=1,synapse=E_syn)
    #Derivada del error en  y
    d_ref_y=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy)
    nengo.Connection(Err_y,d_ref_y,synapse=E_syn,transform=20)
    nengo.Connection(Err_y,d_ref_y,synapse=t_syn,transform=-20)

    # #Integral del error en y
    In_Err_y = nengo.Ensemble(n_neurons=200, dimensions=1, radius=Sy*2)
    nengo.Connection(Err_y, In_Err_y, transform= kiy, synapse=E_syn)
    nengo.Connection(In_Err_y, In_Err_y,transform=1, synapse=E_syn)

    U_y=nengo.Ensemble(n_neurons=500,dimensions=1,radius=Sy*2)
    nengo.Connection(Err_y,U_y,transform=kpy)
    nengo.Connection(d_ref_y,U_y,transform=kdy)
    nengo.Connection(In_Err_y,U_y,transform=1)
    
    nengo.Connection(U_y,Gy,function=None,synapse=None)
    
    
    
    Phi_aux = nengo.Ensemble(n_neurons=500, dimensions=2, radius=Sphi)
    nengo.Connection(U_y, Phi_aux[0], synapse=t_syn)
    nengo.Connection(At[0], Phi_aux[1], synapse=t_syn)
    

    def phi_d(x):
        u= x[0]
        Theta = x[1] 
        return arctan2(u*cos(Theta),g) # Arctan (u*cos(theta)/g)
    
    def cos_t(x):
        return cos(x)
    
    "Control de Phi"
    Aphi=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sphi)
    Bphi=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sphi)
    
    def Ap_fun(x):
        return [x[1]*t_syn,0]+x
    def Bp_fun(x):
        return [0, x[0]*t_syn]
    nengo.Connection(Bphi,Aphi,function=Bp_fun,synapse=t_syn)
    nengo.Connection(Aphi,Aphi,function=Ap_fun,synapse=t_syn)
    
    En_ref_phi=nengo.Ensemble(n_neurons=500, dimensions=1,radius=Sphi)
    nengo.Connection(Phi_aux,En_ref_phi,function=phi_d,synapse=t_syn)
    
    #Error en phi
    Err_phi=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.05)
    nengo.Connection(Aphi[0],Err_phi,synapse=E_syn)
    nengo.Connection(En_ref_phi,Err_phi,synapse=E_syn,transform=-1)
    
    #Derivada del error
    d_ref_phi=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.5)
    nengo.Connection(En_ref_phi,d_ref_phi,synapse=t_syn,transform=20)
    nengo.Connection(En_ref_phi,d_ref_phi,synapse=E_syn,transform=-20)
    
    D_err_phi=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sphi)
    nengo.Connection(Aphi[1],D_err_phi,synapse=E_syn)
    nengo.Connection(d_ref_phi,D_err_phi,synapse=E_syn,transform=-1)


    
    #Control de phi
    U_phi=nengo.Ensemble(n_neurons=500,dimensions=1,radius=0.5)#Radius=1.5
    nengo.Connection(Err_phi,U_phi,transform=-kpphi)
    nengo.Connection(D_err_phi,U_phi,transform=-kdphi)

    
    #Conexion con B
    nengo.Connection(U_phi,Bphi,function=None,transform=None)

    #Conecxion final de u_y a B en la dinamica de Y
   
    
    #Conectamos desde el combinado aplicando la función # g tan(APhi[0])/cos(At[0])
    def RetroPhi_Y(x):
        phi= x[0]
        Theta = x[1] 
        return g*tan(phi) / cos(Theta)
    Retro_auxiliar = nengo.Ensemble(n_neurons=200,dimensions=2,radius=Sphi)
    nengo.Connection(Aphi[0], Retro_auxiliar[0], synapse=t_syn)
    nengo.Connection(At[0], Retro_auxiliar[1], synapse=t_syn)
    nengo.Connection(Retro_auxiliar, U_y, function=RetroPhi_Y, synapse=t_syn)
    nengo.Connection(U_y,Gy,function=None,synapse=None)
    #################################################################################

    
    "Control de Posicion z"
    # theta_r=nengo.Node(lambda x: 5*pi/180*cos(x))
    CC=nengo.Ensemble(n_neurons=200,dimensions=3,neuron_type=nengo.Direct())
    nengo.Connection(Aphi[0],CC[1])
    nengo.Connection(At[0],CC[2])
    #Representacion de la forma f' y g' de las matrices en espacio de estados  de x
    Fz=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sz)
    Gz=nengo.Ensemble(n_neurons=200,dimensions=1,radius=28.5)
    G=nengo.Node(g)
    def fz_fun(x):
        return [x[1]*t_syn,0]+x 
    def gz_fun(x):
        return [0,x[0]*t_syn]
    def upsilon_z(x):
        return (-x[0]*cos(0)*cos(0))/m +g 
    nengo.Connection(CC, Gz, function=upsilon_z)
    #nengo.Connection(Gz,CC[0])
    nengo.Connection(Gz,Fz,synapse=t_syn,function=gz_fun)
    nengo.Connection(Fz,Fz,synapse=t_syn,function=fz_fun)
    
     #Referencia
    ref_z=nengo.Node(lambda x, k=1, centro=4: -3/ (1 + np.exp(-k * (x - centro))))
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

    # #Integral del error en y
    In_Err_z = nengo.Ensemble(n_neurons=200, dimensions=1, radius=Sy*2)
    nengo.Connection(Err_z, In_Err_z, transform= kiz, synapse=E_syn)
    nengo.Connection(In_Err_z, In_Err_z,transform=1, synapse=E_syn)
    
    
   
    ########### Eq. (7)
    u_z=nengo.Ensemble(n_neurons=300,dimensions=1,radius=28.5)
    nengo.Connection(Err_z,u_z,transform=kpz)
    nengo.Connection(D_Err_z,u_z,transform=kdz)
    nengo.Connection(In_Err_z,u_z,transform=1)

    u=nengo.Ensemble(n_neurons=500,dimensions=3,neuron_type=nengo.Direct())
    nengo.Connection(u_z,u[0])
    nengo.Connection(Aphi[0],u[1])
    nengo.Connection(At[0],u[2])
    ########### Eq. (6)
    def u_fun(x): 
        return (x[0]+g)*m/(cos(0)*cos(0))
    nengo.Connection(u,CC[0],function=u_fun,synapse=None)

    
    #Simulacion y
    Fy_p=nengo.Probe(Fy,synapse=t_syn)
    Ry_p=nengo.Probe(En_refy,synapse=t_syn)
    Ery_p=nengo.Probe(Err_y,synapse=t_syn)
    Uy_p=nengo.Probe(U_y,synapse=t_syn)
    Aphi_p=nengo.Probe(Aphi,synapse=t_syn)
    Ephi_p=nengo.Probe(Err_phi,synapse=t_syn)
    DEphi_p=nengo.Probe(D_err_phi,synapse=t_syn)
    Uphi_p=nengo.Probe(U_phi,synapse=t_syn)
    Rphi_p=nengo.Probe(En_ref_phi,synapse=t_syn)
    #######Coseno_t=nengo.Probe(Cos_theta,synapse=t_syn)
    #Simulacion z
    Fz_p=nengo.Probe(Fz,synapse=t_syn)
    Rz_p=nengo.Probe(En_refz,synapse=t_syn)
    Erz_p=nengo.Probe(Err_z,synapse=t_syn)
    DErr_z=nengo.Probe(D_Err_z,synapse=t_syn)
    U_p=nengo.Probe(u,synapse=t_syn)
    Gz_p=nengo.Probe(Gz,synapse=t_syn)
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

    with nengo.Simulator(model) as sim:
        sim.run(60)
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
    
    #Calculo de maximos para la saturacion de los ensambles
    Ery_data=sim.data[Ery_p]
    Uy_data=sim.data[Uy_p]
    
    m_Ery=max(Ery_data)
    m_Uy=max(Uy_data)

    print("El error maximo en y es :",m_Ery)
    print("La entrada maxima en U_y es:",m_Uy)
    
    Erphi_data=sim.data[Ephi_p]
    DErphi_data=sim.data[DEphi_p]
    Uphi_data=sim.data[Uphi_p]
    #Datos en phi
    m_Erphi=max(Erphi_data)
    m_dErphi=max(DErphi_data)
    m_Uphi=max(Uphi_data)
    print("El error maximo en phi es :",m_Erphi)
    print("La derivada del error maxima en phi es:", m_dErphi)
    print("La entrada maxima en U_phi es:",m_Uphi)

    # #Exportacion de datos a csv 
    data = sim.data[Fy_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_y.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_y.csv")
    # #Exportacion de datos a csv 
    data = sim.data[Aphi_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_phi.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_phi.csv")

    #Exportacion de datos a csv 
    data = sim.data[Uy_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_Uy.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_inputsY.csv")
    #Exportacion de datos a csv 
    data = sim.data[Ux_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_Ux.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_inputsX.csv")
    #Exportacion de datos a csv 
    data = sim.data[U_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_Uz.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_inputsZ.csv")

    #Exportacion de datos a csv 
    data = sim.data[Ut_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_Tautheta.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_inputsTheta.csv")
    #Exportacion de datos a csv 
    data = sim.data[Uphi_p]
    export_data = np.column_stack((t, data))
    np.savetxt('simulink_data_TauPhi.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_inputsPhi.csv")



    
    # Er_data=sim.data[Erz_p]
    # DEr_data=sim.data[DErr_z]
    # #U_data=sim.data[U_p]
    # m_Er=max(Er_data)
    # m_dEr=max(DEr_data)
    # #m_U=max(U_data)
    # print("El error maximo es :",m_Er)
    # print("La derivada del error maxima es:", m_dEr)
    # print("El empuje maximo es:",m_U)
    # #Exportacion de datos a csv 
    # data = sim.data[Fz_p]
    # export_data = np.column_stack((t, data))

    np.savetxt('simulink_data_z.csv', 
               export_data, 
               delimiter=',', 
               header='Time,Position', 
               comments='')
    print("Datos exportados a simulink_data_z.csv")
    

    def plot_X(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[Fx_p][:,0], label="Position X", color='red')
        plt.plot(t, data[Rx_p], label="Ref x", color='green')
        #plt.plot(t, data[Ux_p], label="U_x ", color='blue')

    
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
        plt.savefig('Position X', format="pdf", bbox_inches='tight')
        plt.show()   
        
        
    
    plot_X(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-1, ymax=2)
    
    def plot_t(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
         
         plt.figure(figsize=(9, 10))
         plt.plot(t, data[At_p][:,0], label="Position theta", color='red')
         plt.plot(t, data[Rt_p], label="Ref theta", color='green')
         #plt.plot(t, data[Et_p], label="Err theta ", color='blue')

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
         plt.savefig('Position Theta', format="pdf", bbox_inches='tight') 
         plt.show()  
     
    plot_t(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-pi*10/180, ymax=pi*10/180)
    
    def plot_Y(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[Fy_p][:,0], label="Position y", color='red')
        plt.plot(t, data[Ry_p], label="Ref y", color='green')
        # plt.plot(t, data[Uy_p], label="U_y", color='blue')
        # plt.plot(t, data[Coseno_t], label="Ref y", color='gray')

    
        plt.xlabel("Time [s]", fontsize=32)
        plt.ylabel("Position y [m]", fontsize=32)
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
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Y'
        plt.savefig('Position Y', format="pdf", bbox_inches='tight')
        plt.show()   
    
    plot_Y(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-1, ymax=2)
    
    def plot_phi(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
         
         plt.figure(figsize=(9, 10))
         plt.plot(t, data[Aphi_p][:,0], label="Position t", color='red')
         plt.plot(t, data[Rphi_p], label="Ref t", color='green')


         plt.xlabel("Time [s]", fontsize=32)
         plt.ylabel("Position Phi [rad]", fontsize=32)
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
         
         # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Phi'
         plt.savefig('Position Phi', format="pdf", bbox_inches='tight')
         plt.show()   
     
    plot_phi(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-pi*30/180, ymax=pi*30/180)
    
    def plot_Z(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[Fz_p][:,0], label="Position z", color='red')
        plt.plot(t, data[Rz_p], label="Ref z", color='green')

    
        plt.xlabel("Time [s]", fontsize=32)
        plt.ylabel("Position z [m]", fontsize=32)
        plt.grid(True)
    
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
        
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Z'
        plt.savefig('Position Z', format="pdf", bbox_inches='tight')
        ### plt.show()   
    
    plot_Z(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-3.5, ymax=1)
    
    def plot_U(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[U_p][:,0], label="Thrust [N]", color='red')

    
        plt.xlabel("Time [s]", fontsize=32)
        plt.ylabel("F [N]", fontsize=32)
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

        # plt.show()
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Z'
        # plt.savefig(file, format="pdf", bbox_inches='tight')   
    
    plot_U(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=EjeX, ymin=-15, ymax=28.5)
   
 