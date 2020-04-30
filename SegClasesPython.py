
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Particle:

    #Atributos
    cargada = True

    #Instancias

    def __init__(self, x, y, z, vx, vy, vz, m, carga):

        self.X=x
        self.Y=y
        self.Z=z
        self.VX=vx
        self.VY=vy
        self.VZ=vz
        self.M=m
        self.Carga=carga
    

    def Fuerza(self, Ex, Ey, Ez):

        BZ = 10.0
        Fx = self.Carga*(Ex + self.VY*BZ)
        Fy = self.Carga*(Ey - self.VX*BZ)
        Fz = self.Carga*Ez

        return Fx,Fy,Fz

        
    def Vel_evol(self, ax, ay, az, t):

        self.VX = self.VX + ax*t
        self.VY = self.VY + ay*t
        self.VZ = self.VZ + az*t


    def Pos_evol(self, ax, ay, az, t):

       self.X = self.X + (self.VX * t) + (0.5*ax*t*t)
       self.Y = self.Y + (self.VY * t) + (0.5*ay*t*t)
       self.Z = self.Z + (self.VZ * t) + (0.5*az*t*t)
       

Part1 = Particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 1)
Part2 = Particle(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, -1)



x1 = []
y1 = []
z1 = []

x2 = []
y2 = []
z2 = []

for i in range(0, 10000):
    if ((Part2.X-Part1.X)**(2) + (Part2.Y-Part1.Y)**(2) + (Part2.Z-Part1.Z)**(2)) == 0: break

    Ex2 = (Part2.Carga*(Part1.X-Part2.X))/(4*np.pi*(8.8541e-12)*((Part1.X-Part2.X)**(2) + (Part1.Y-Part2.Y)**(2) + (Part1.Z-Part2.Z)**(2))**(3/2))
    Ey2 = (Part2.Carga*(Part1.Y-Part2.Y))/(4*np.pi*8.8541e-12*((Part1.X-Part2.X)**(2) + (Part1.Y-Part2.Y)**(2) + (Part1.Z-Part2.Z)**(2))**(3/2))
    Ez2 = (Part2.Carga*(Part1.Z-Part2.Z))/(4*np.pi*8.8541e-12*((Part1.X-Part2.X)**(2) + (Part1.Y-Part2.Y)**(2) + (Part1.Z-Part2.Z)**(2))**(3/2))
    
    Ex1 = (Part1.Carga*(-Part1.X+Part2.X))/(4*np.pi*8.8541e-12*((Part2.X-Part1.X)**(2) + (Part2.Y-Part1.Y)**(2) + (Part2.Z-Part1.Z)**(2))**(3/2))
    Ey1 = (Part1.Carga*(-Part1.Y+Part2.Y))/(4*np.pi*8.8541e-12*((Part2.X-Part1.X)**(2) + (Part2.Y-Part1.Y)**(2) + (Part2.Z-Part1.Z)**(2))**(3/2))
    Ez1 = (Part1.Carga*(-Part1.Z+Part2.Z))/(4*np.pi*8.8541e-12*((Part2.X-Part1.X)**(2) + (Part2.Y-Part1.Y)**(2) + (Part2.Z-Part1.Z)**(2))**(3/2))

    Fx, Fy, Fz =Part1.Fuerza(Ex2, Ey2, Ez2)
    Fx2, Fy2, Fz2 =Part2.Fuerza(Ex1, Ey1, Ez1)
    
    Part1.Vel_evol(Fx/Part1.M, Fy/Part1.M, Fz/Part1.M, 0.1)
    Part1.Pos_evol(Fx/Part1.M, Fy/Part1.M, Fz/Part1.M, 0.1)
    Part2.Vel_evol(Fx2/Part2.M, Fy2/Part2.M, Fz2/Part2.M, 0.1)
    Part2.Pos_evol(Fx2/Part2.M, Fy2/Part2.M, Fz2/Part2.M, 0.1)
 

    x1.append(Part1.X)
    y1.append(Part1.Y)
    z1.append(Part1.Z)

    x2.append(Part2.X)
    y2.append(Part2.Y)
    z2.append(Part2.Z)
    


fig = plt.figure(figsize = (10,10))
ax = Axes3D(fig)
ax.plot(x1,y1,z1, label = "Particle 1", color = 'red')
ax.plot(x2,y2,z2, label = "Particle 2", color = 'blue')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_title('Evolucion particulas cargadas en campo electrico')
plt.legend()
plt.show()

