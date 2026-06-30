import nengo
import numpy as np

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
model=nengo.Network(seed=40)
#Saturacion de neuronas
Sx=5
St=pi*40/180
Sx=5
Sz=15
Sy=5
Sphi=pi*40/180
#Ganancias
kpx=2.7#2.2
kdx=0.22 #0.2
kix = 0.2 #0.2

kpt=75 #75.5
kdt=20 #20

kpz=40
kdz=5

kpphi=75 #Tenia 130, 75
kdphi=8 #Tenia 55, 8
kiphi = 0.0#

kpy=80 #Tenia 3, 30
kdy=50 #Tenia 2, 8
kiy = 0.1 #
#
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
    nengo.Connection(Fx[0],Err_x,synapse=E_syn)
    nengo.Connection(En_refx,Err_x,transform=-1,synapse=E_syn)
    #Derivada del error en  x 
    D_err_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx)
    nengo.Connection(Err_x,D_err_x,synapse=E_syn,transform=20)
    nengo.Connection(Err_x,D_err_x,synapse=t_syn,transform=-20)

    #Integral del error en x
    In_Err_x = nengo.Ensemble(n_neurons=200, dimensions=1, radius=Sx)
    nengo.Connection(Err_x, In_Err_x, transform= 0.01, synapse=E_syn)
    nengo.Connection(In_Err_x, In_Err_x,transform=1, synapse=E_syn)
    

    #Control, para la obtencion de u_x=g*tan(theta_d)
    U_x=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sx*2)
    nengo.Connection(Err_x,U_x,transform=-kpx)
    nengo.Connection(D_err_x,U_x,transform=-kdx)
    nengo.Connection(In_Err_x,U_x,transform=-kix)

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
    # En_ref_t=nengo.Ensemble(n_neurons=200, dimensions=1,radius=St,neuron_type=nengo.Direct()) 
    # nengo.Connection(ref_t,En_ref_t,synapse=t_syn)
    
    #Error en theta
    Err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=0.1)
    nengo.Connection(At[0],Err_t,synapse=E_syn)
    nengo.Connection(ref_t,Err_t,synapse=E_syn,transform=-1)

    #Derivada del error   
    d_ref_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1)
    nengo.Connection(ref_t,d_ref_t,synapse=t_syn,transform=20)
    nengo.Connection(ref_t,d_ref_t,synapse=E_syn,transform=-20)

    D_err_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=St)
    nengo.Connection(At[1],D_err_t,synapse=E_syn)
    nengo.Connection(d_ref_t,D_err_t,synapse=E_syn,transform=-1)
     
    #Control de theta
    U_t=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1.5)
    nengo.Connection(Err_t,U_t,transform=-kpt)
    nengo.Connection(D_err_t,U_t,transform=-kdt) 
     
    #Conexion con B
    nengo.Connection(U_t,Bt,function=None,transform=None)

    #Conecxion final de u_x a B en la dinamica de X
    def RetroTheta_X(x): 
        return -g*tan(x)
    nengo.Connection(At[0],U_x,function=RetroTheta_X) # -g tan(A[0])
    nengo.Connection(U_x,Gx,function=None,synapse=None)
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
    ref_y=nengo.Node(lambda x, k=1, centro=5:1.8 / (1 + np.exp(-k * (x - centro))))
    En_refy=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy,neuron_type=nengo.Direct())
    nengo.Connection(ref_y,En_refy)
    
    #Errores en y
    Err_y=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy)
    nengo.Connection(Fy[0],Err_y,synapse=E_syn)
    nengo.Connection(En_refy,Err_y,transform=-1,synapse=E_syn)
    #Derivada del error en  y
    d_ref_y=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy)
    nengo.Connection(Err_y,d_ref_y,synapse=E_syn,transform=20)
    nengo.Connection(Err_y,d_ref_y,synapse=t_syn,transform=-20)

    # #Integral del error en y
    In_Err_y = nengo.Ensemble(n_neurons=500, dimensions=1, radius=1.5)
    nengo.Connection(Err_y, In_Err_y, transform= 0.01, synapse=E_syn)
    nengo.Connection(In_Err_y, In_Err_y,transform=1, synapse=E_syn)

    #Control, para la obtencion de u_x=-g*tan(phi)/cos(theta)
    U_y=nengo.Ensemble(n_neurons=200,dimensions=1,radius=Sy*2)
    nengo.Connection(Err_y,U_y,transform=-kpy)
    nengo.Connection(d_ref_y,U_y,transform=-kdy)
    nengo.Connection(In_Err_y,U_y,transform=-kiy)
    
    #########nengo.Connection(U_y,Gy,function=None,synapse=None)
    
    
     # 1. Creamos un ensemble intermedio de 2 dimensiones para combinar U_y y At[0]
    Phi_aux = nengo.Ensemble(n_neurons=500, dimensions=2, radius=Sphi)
    # 2. Conectamos U_y a la primera dimensión [0] de nuestro nuevo ensemble
    nengo.Connection(U_y, Phi_aux[0], synapse=t_syn)
    # 3. Conectamos At[0] a la segunda dimensión [1] de nuestro nuevo ensemble
    nengo.Connection(At[0], Phi_aux[1], synapse=t_syn)
    ##OBSERVE QUE LOS VALORES DE U_Y Y THETA SON CONSTANTES EN ESTA DINAMICA ##########

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

    # #Integral del error en x
    # In_Err_phi = nengo.Ensemble(n_neurons=500, dimensions=1, radius=1.5)
    # nengo.Connection(Err_phi, In_Err_phi, transform= 0.01, synapse=E_syn)
    # nengo.Connection(In_Err_phi, In_Err_phi,transform=1, synapse=E_syn)
    
    #Control de phi
    U_phi=nengo.Ensemble(n_neurons=200,dimensions=1,radius=1.5)
    nengo.Connection(Err_phi,U_phi,transform=-kpphi)
    nengo.Connection(D_err_phi,U_phi,transform=-kdphi)
    # nengo.Connection(In_Err_phi,U_phi,transform=-kiphi)
    
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

    #"Control de Posicion z"
    
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
        return [0,((x[0]*cos(x[1])*cos(x[2]))/m -g)*t_syn]
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
        return x[0]/(cos(x[1])*cos(x[2]))
   
    U=nengo.Ensemble(n_neurons=100,dimensions=1,radius=28.5)
    nengo.Connection(G,U,transform=m)
    nengo.Connection(Err_z,U,transform=-kpz)
    nengo.Connection(D_Err_z,U,transform=-kdz)
    CC2=nengo.Ensemble(n_neurons=200,dimensions=3,neuron_type=nengo.Direct())
    nengo.Connection(U,CC2[0])
    nengo.Connection(Aphi[0],CC2[1])
    nengo.Connection(At[0],CC2[2])
    nengo.Connection(CC2,Gz,function=u_fun,synapse=None)
