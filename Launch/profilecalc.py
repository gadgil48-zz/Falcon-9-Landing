import math, numpy
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

global dm, m0
dm = -(75.5368+175.5873)*9 #rate of fuel consumption kg/s
m0 = 486.931*(10.0**3.0) #mass at 100m/s
m01 = 548.759*(10.0**3.0) #mass at 0m/s
mwet = 424.6*(10.0**3.0) #mass of first stage fuel tank with fuel
mdry = 15*(10.0**3.0) #mass of first stage fuel tank without fuel

fw = (mwet-(m01-m0)-mdry) #total mass of fuel used after 100m/s
tf = -fw/dm #time of burn

def rho(y,t): #atmospheric density
    return 1.225*math.exp(-9.81*0.0289*y/(8.314*288.15))

def Fric(y,v,t): #aerodynamic drag
    return 0.5*rho(y,t)*v*v*0.3*22.8

MaxT = 820*9*(10.0**3.0) #Max Thrust
r = 6371000 #Earth radius

def g(y): #gravity
    return 3.986*(10.0**14.0)/((y+r)*(y+r))

def dv(y,psi,v,t): #acceleration
    return MaxT/(m0+dm*t) - g(y)*math.cos(psi) - Fric(y,v,t)/(m0+dm*t)

def dpsi(y,psi,v): #change in pitch angle
    return g(y)*math.sin(psi)/v

def model(v, a, b): #fitting model
    return a*numpy.arctan(b*(v))

y0 = 1381.1447 #height in m at 100m/s
v0 = 100 #initial velocity
psi0 = math.radians(2.5) #varied quantity, pitch-over

incr = 0.1

#storage arrays
H = [] #Height
V = [] #Velocity
Psi = [] #Pitch angle(actually 90-pitch angle)
T = [] #Time

t1=0.0
while t1<tf: #Simple Euler method for solving
    y1 = y0 + v0*math.cos(psi0)*incr #new value for height
    psi1 = psi0 + dpsi(y0,psi0,v0)*incr #new value for pseudo-pitch
    v1 = v0 + dv(y0,psi0,v0,t1)*incr #new value for velocity

    #setting up for next loop
    y0 = y1
    psi0 = psi1
    v0 = v1

    #storing
    H.append(y0)
    V.append(v0)
    Psi.append(math.degrees(psi0))
    T.append(t1)
    t1+=incr
    


popt, pcov = curve_fit(model, V, Psi, bounds = ([0,0],[100,1]))
c = 0

while numpy.abs(model(100+c, *popt)-math.radians(1.5))>0.001 and c>=-100:
    c += -0.001

params = popt

print("Parameters for model: a*arctan(b*(v+c)) \n")
print("a: %f" % params[0])
print("b: %f" % params[1])
print("c: %f" % c)
print("Final Height: %f \t" % y0)
print("Final velocity: %f \t" % v0)
print("Final pitch: %f \t" % (90-math.degrees(psi0)))

tAp = v0*math.cos(psi0)/g(y0)
print("Time to Apoapsis: %f mins\t" % (tAp/60))
h = v0*math.cos(psi0)*tAp - 0.5*g(y0)*tAp*tAp
print("Predicted Apoapsis: %f m\t" % (y0+h))
