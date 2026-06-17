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
e=np.exp
#Modelo
t_syn=0.1
E_syn=0.05
model=nengo.Network(label="Quadrirotor_Z_control")
#Saturacion de neuronas
Sx=5
St=1
Sz=15
#Ganancias
kpx=1.6
kdx=2
kpt=120
kdt=8
kpz=40;
kdz=5;
#Agulos
theta=0;
phi=0;
#Constantes de control 
epsilon = .0000000001
with model:
    "Control de Posicion z"
    #Representacion de la forma f' y g' de las matrices en espacio de estados  de x
    Fz=nengo.Ensemble(n_neurons=500,dimensions=2,radius=Sz)
    Gz=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sz)
    G=nengo.Node(g)
    def fz_fun(x):
        return [x[1]*t_syn,0]+x 
    def gz_fun(x):
        return [0,(x[0]/m -g)*t_syn]

    nengo.Connection(Gz,Fz,synapse=t_syn,function=gz_fun)
    nengo.Connection(Fz,Fz,synapse=t_syn,function=fz_fun)
    
     #Referencia
    ref_z=nengo.Node(lambda x, k=1, centro=4: 3/ (1 + np.exp(-k * (x - centro))))
    En_refz=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sz,neuron_type=nengo.Direct())
    nengo.Connection(ref_z,En_refz)
    
    #Errores en z
    Err_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=2)
    nengo.Connection(Fz[0],Err_z,synapse=E_syn)
    nengo.Connection(En_refz,Err_z,transform=-1,synapse=E_syn)
    #Derivada de z deseado
    d_ref_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=Sz)
    nengo.Connection(En_refz,d_ref_z,synapse=E_syn,transform=-20)
    nengo.Connection(En_refz,d_ref_z,synapse=t_syn,transform=20)
    
    #Derivadas del error en z
    D_Err_z=nengo.Ensemble(n_neurons=100,dimensions=1,radius=2.5)
    nengo.Connection(Fz[1],D_Err_z,synapse=E_syn)
    nengo.Connection(d_ref_z,D_Err_z,transform=-1,synapse=E_syn)
    
    
    #Control, para la obtencion de u

   
    U=nengo.Ensemble(n_neurons=200,dimensions=1,radius=28.5)
    nengo.Connection(G,U,transform=m)
    nengo.Connection(Err_z,U,transform=-kpz)
    nengo.Connection(D_Err_z,U,transform=-kdz)
    
    #Theta
    t=nengo.Node(.5)
    theta=nengo.Ensemble(n_neurons=200,dimensions=1,neuron_type=nengo.Direct())    
    nengo.Connection(t,theta)
    
    def u_fun(x):
    
        return x[1]/0.5

    #Canal de conexión 
    CC=nengo.Ensemble(n_neurons=400,dimensions=2,neuron_type=nengo.Direct())
    nengo.Connection(theta,CC[0])
    nengo.Connection(U,CC[1])
    U_prime=nengo.Ensemble(n_neurons=200,dimensions=1,radius=28.5)
    nengo.Connection(CC,U_prime,function=u_fun)
    # nengo.Connection(U,Gz,function=None,synapse=None)
    
    
    #CANAL DE CONEXIÓN 2
    CC2=nengo.Ensemble(n_neurons=400,dimensions=2,neuron_type=nengo.Direct())
    nengo.Connection(theta,CC2[0])
    nengo.Connection(U_prime,CC2[1])
    #Producto 
    def prod(x):
        return x[0]*x[1]
    nengo.Connection(CC2,Gz,function=prod)
    #Simulacion
    Fz_p=nengo.Probe(Fz,synapse=t_syn)
    Rz_p=nengo.Probe(En_refz,synapse=t_syn)
    Erz_p=nengo.Probe(Err_z,synapse=t_syn)
    DErr_z=nengo.Probe(D_Err_z,synapse=t_syn)
    U_p=nengo.Probe(U,synapse=t_syn)
    
   
    with nengo.Simulator(model) as sim:
        sim.run(20)
    t = sim.trange()
    #Calculo de maximos para la saturacion de los ensambles
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
    # print("Datos exportados a simulink_data.csv")
    
    def plot_Z(data, num_xticks=5, num_yticks=5, xmin=None, xmax=None, ymin=None, ymax=None):
        
        plt.figure(figsize=(9, 10))
        plt.plot(t, data[Fz_p][:,0], label="Position z", color='red')
        plt.plot(t, data[Rz_p], label="Ref z", color='green')
        plt.plot(t, data[Erz_p], label="Err z ", color='blue')
    
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
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Z'
        # plt.savefig(file, format="pdf", bbox_inches='tight')   
    
    plot_Z(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=20, ymin=-10, ymax=10)
    
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
        
        # file = '/Users/Usuario/Documents/IA/ESTANCIA/Quad/' + 'Position Z'
        # plt.savefig(file, format="pdf", bbox_inches='tight')   
    
    plot_U(sim.data, num_xticks=6, num_yticks=6, xmin=-1, xmax=20, ymin=-15, ymax=30)