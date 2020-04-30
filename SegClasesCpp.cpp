#include <iostream>
#include <cmath>

using namespace std;

class Particula{

public:

  int Carga;
  float X,Y,Z;
  float VX,VY,VZ;
  float M;

  void SetValue(float, float, float, float, float, float, float, int);
  float Fuerza(float, float, float, int);
  float Vel_evo(float, float, float, float);
  float Pos_evol(float, float, float, float);
    
}Par1,Par2;

void Particula::SetValue(float x, float y, float z, float vx, float vy, float vz, float m, int c){

  X = x;
  Y = y;
  Z = z;
  VX = vx;
  VY = vy;
  VZ = vz;
  M = m;
  Carga = c;
} 

float Particula::Fuerza(float Ex, float Ey, float Ez, int i){

  float BZ = 10;
  float Fx = Carga*(Ex + VY*BZ);
  float Fy = Carga*(Ey - VX*BZ);
  float Fz = Carga*Ez;

  if (i==0) return Fx;

  else if (i==1) return Fy;

  else if (i==2) return Fz;
    }

float Particula::Vel_evo(float ax, float ay, float az, float t){

  VX = VX + ax*t;
  VY = VY + ay*t;
  VZ = VZ + az*t;
}

float Particula::Pos_evol(float ax, float ay, float az, float t){

  X = X + (VX * t) + (0.5*ax*t*t);
  Y = Y + (VY * t) + (0.5*ay*t*t);
  Z = Z + (VZ * t) + (0.5*az*t*t);

 }


int main(){


  Par1.SetValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 1);
  Par2.SetValue(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, -1);

  for(int j=0; j<=10000; j++){

    
    if ((pow((Par2.X-Par1.X),2) + pow((Par2.Y-Par1.Y),2) + pow((Par2.Z-Par1.Z),2))==0) break;

    float Ex2 = (Par2.Carga*(Par1.X-Par2.X))/(4*M_PI*(8.8541e-12)*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
    float Ey2 = (Par2.Carga*(Par1.Y-Par2.Y))/(4*M_PI*8.8541e-12*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
    float Ez2 = (Par2.Carga*(Par1.Z-Par2.Z))/(4*M_PI*8.8541e-12*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
    
    float Ex1 = (Par1.Carga*(-Par1.X+Par2.X))/(4*M_PI*8.8541e-12*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
    float Ey1 = (Par1.Carga*(-Par1.Y+Par2.Y))/(4*M_PI*8.8541e-12*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
    float Ez1 = (Par1.Carga*(-Par1.Z+Par2.Z))/(4*M_PI*8.8541e-12*pow((pow((Par1.X-Par2.X),2) + pow((Par1.Y-Par2.Y),2) + pow((Par1.Z-Par2.Z),2)),3/2));
	  


    float Fx = Par1.Fuerza(Ex2, Ey2, Ez2, 0);
    float Fy = Par1.Fuerza(Ex2, Ey2, Ez2, 1);
    float Fz = Par1.Fuerza(Ex2, Ey2, Ez2, 2);
    float Fx2 = Par2.Fuerza(Ex1, Ey1, Ez1, 0);
    float Fy2 = Par2.Fuerza(Ex1, Ey1, Ez1, 1);
    float Fz2 = Par2.Fuerza(Ex1, Ey1, Ez1, 2);
    

    Par1.Vel_evo(Fx/Par1.M, Fy/Par1.M, Fz/Par1.M, 0.1);
    Par1.Pos_evol(Fx/Par1.M, Fy/Par1.M, Fz/Par1.M, 0.1);
    Par2.Vel_evo(Fx2/Par2.M, Fy2/Par2.M, Fz2/Par2.M, 0.1);
    Par2.Pos_evol(Fx2/Par2.M, Fy2/Par2.M, Fz2/Par2.M, 0.1);

    cout << Par1.X << "," << Par1.Y << "," << Par1.Z << "," << Par2.X << "," << Par2.Y << "," << Par2.Z <<  endl; 


  }

  return 0;

}
