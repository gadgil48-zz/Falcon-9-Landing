import krpc
import numpy as np
#import time
import Global

conn = Global.conn
vessel = conn.space_center.active_vessel

def norm(x):
    return np.linalg.norm(x)

def r2d(x):
    return x*180.0/np.pi
    
def d2r(x):
    return x*np.pi/180.0

#function to calculate distance between 2 (lat,lon) coords
def db2(lat1, lon1, lat2, lon2):
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(0.5*dlat)**2.0 + np.cos(lat1)*np.cos(lat2)*(np.sin(0.5*dlon)**2.0)
    c = 2.0*np.arcsin(min(1.0, np.sqrt(a)))
    d = vessel.orbit.body.equatorial_radius*c
    return d
    
#function to calculate (lat, long) from state vectors
def latlon(x):
    lon = np.arctan2(x[1],x[0])
    p = (x[0]**2.0 + x[1]** 2.0)**0.5
    lat = np.arctan2(x[2],p)
    lat = r2d(lat)
    lon = r2d(lon)
    return np.array((lat, lon))
    
def Toggle(init_x, init_v):
    bmu = vessel.orbit.body.gravitational_parameter
    br = vessel.orbit.body.equatorial_radius
    
    r = init_x
    v = init_v
    
    r = [r[0], r[2], r[1]]
    
    v = [v[0], v[2], v[1]]

    #calculate orbital elements from state vectors
    h = np.cross(r, v)
    e = np.subtract(np.divide(np.cross(v, h), bmu), np.divide(r, norm(r)))    
    ec = norm(e)

    n = np.transpose([-h[1], h[0], 0])

    if np.dot(r, v)>=0:
        nu = np.arccos(np.dot(e, r)/(ec*norm(r)))
    else:
        nu = 2*np.pi - np.arccos(np.dot(e, r)/(ec*norm(r)))

    i = np.arccos(h[2]/norm(h))
    if n[1]>=0:
        RAAN = np.arccos(n[0]/norm(n))
    else:
        RAAN = 2*np.pi - np.arccos(n[0]/norm(n))

    if e[2]>=0:
        w = np.arccos(np.dot(n, e)/(ec*norm(n)))
    else:
        w = 2*np.pi - np.arccos(np.dot(n, e)/(ec*norm(n)))

    a = (2.0/norm(r) - (norm(v)**2.0)/bmu)**-1
    
    elem = [a, ec, i, w, RAAN, nu]
    
    #calculate new true anomaly when distance to centre of earth is 6371km on the orbit
    elem[5] = -np.arccos(((1.0-elem[1]**2.0)*(elem[0]/br)-1.0)/elem[1])
    
    #convert back to state vectors
    X = np.cos(elem[4])*np.cos(elem[3]+elem[5]) - np.sin(elem[4])*np.sin(elem[3]+elem[5])*np.cos(elem[2])
    Y = np.sin(elem[4])*np.cos(elem[3]+elem[5]) + np.cos(elem[4])*np.sin(elem[3]+elem[5])*np.cos(elem[2])
    Z = np.sin(elem[2])*np.sin(elem[3]+elem[5])
    
    X *= br
    Y *= br
    Z *= br
    
    R = [X, Y, Z]
    
    return R
