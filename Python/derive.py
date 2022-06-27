from sympy import *

Ix = 1
Iy = 1
Iz = 1
g = 9.81
m = 0.5
dy = .1
dx = .1

def T(f, dx, dy):

    return f*(sqrt(dx**2 + dy**2))

u,v,w,p,q,r,phi,theta,psi,x,y,z,f1,f2,f3,f4 = symbols('u,v,w,p,q,r,phi,theta,psi,x,y,z,f1,f2,f3,f4')
Fz = f1+f2+f3+f4
L = f1*dy - f2*dy - f3*dy + f4*dy
M = f1*dx - f2*dx + f3*dx - f4*dx
N = -T(f1,dx,dy) - T(f2,dx,dy) + T(f3,dx,dy) + T(f4,dx,dy)
f = Matrix([[-g*sin(theta) + r*v - q*w], 
            [g*sin(phi)*cos(theta) - r*u + p*w], 
            [(1/m)*(-Fz) + g*cos(phi)*cos(theta) + q*u - p*v], 
            [(1/Ix)*(L+(Iy-Iz)*q*r)], 
            [(1/Iy)*(M+(Iz-Ix)*p*r)], 
            [(1/Iz)*(N+(Ix-Iy)*p*q)], 
            [p + (q*sin(phi) + r*cos(phi))*tan(theta)], 
            [q*cos(phi) - r*sin(phi)], 
            [(q*sin(phi) + r*cos(phi))*(1/cos(theta))], 
            [cos(theta)*cos(psi)*u + (-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi))*v + (sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi))*w], 
            [cos(theta)*sin(psi)*u + (-cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi))*v + (-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi))*w], 
            [-1*(-sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w)]])
print(f)
fnum = lambdify((u,v,w,p,q,r,phi,theta,psi,x,y,z,f1,f2,f3,f4), f)
A = lambdify((u,v,w,p,q,r,phi,theta,psi,x,y,z, f1, f2, f3, f4), f.jacobian([u,v,w,p,q,r,phi,theta,psi,x,y,z]))
B = lambdify((u,v,w,p,q,r,phi,theta,psi,x,y,z, f1, f2, f3, f4), f.jacobian([f1,f2,f3,f4]))
print(fnum(0,0,0,0,0,0,0,0,0,0,0,0,9.81/8,9.81/8,9.81/8,9.81/8))