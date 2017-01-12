import krpc
import numpy
import time
import threading
import Global
import PIDLoop
conn = Global.conn
vessel = Global.vessel

#include Launch if you want to do a full mission
#import Launch
#Prep-work
vessel.control.rcs = True
vessel.control.throttle = 0.0

bref = vessel.orbit.body.reference_frame

altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'surface_altitude')

target = Global.target

q = []
w = []
e = []
r = []
smode1 = []

i = False
o = False
Dir = (0.0,-1.0,0.0)
Dir = conn.space_center.transform_direction(Dir, vessel.surface_velocity_reference_frame, vessel.surface_reference_frame)
Dir = numpy.asarray(Dir)
Dir[0] = 0
Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
Dir = (Dir[0], Dir[1], Dir[2])
mode = 3
smode = 0

q.insert(0, i)
w.insert(0, o)
e.insert(0, Dir)
smode1.insert(0, smode)

#necessary modules
import Steering_Logic as SAS
import Throttle as T
import Proportional_navigation as PN

s = threading.Thread(target = SAS.Toggle, args=(q, e, smode1,))
s.start()

target_offset = numpy.linalg.norm(target.position(target.orbit.body.reference_frame))-vessel.orbit.body.equatorial_radius-target.flight(target.orbit.body.reference_frame).mean_altitude

h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]

altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'surface_altitude')
vertical_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
surface_speed = conn.add_stream(getattr, vessel.flight(bref), 'speed')

#necessary module
import Predictor as pred

i = True

q.insert(0,i)

while True:
    if altitude()>139000:#change this condition depending on trajectory
        break
    time.sleep(0.01)

#Boost-Back Burn
br = vessel.orbit.body.equatorial_radius

#Section to help with corrective steering for boost-back burn
vespos = vessel.position(bref)
vesvpos = vessel.velocity(bref)
tpos = target.position(bref)
tpos = [tpos[0], tpos[2], tpos[1]]
x_1 = pred.Toggle(vespos, vesvpos)

vespos = [vespos[0],vespos[2],vespos[1]]
vespos = numpy.multiply(vespos, br/numpy.linalg.norm(vespos))
n_vec = numpy.cross(vespos, tpos)
n_vec = numpy.divide(n_vec, numpy.linalg.norm(n_vec))
n_vecp = numpy.cross(vespos,x_1)
n_vecp = numpy.divide(n_vecp, numpy.linalg.norm(n_vecp))

if numpy.dot(n_vec, n_vecp)>=0:
    predir = 1
else:
    predir = -1

#Start of Boost-Back Burn
while h>60000:    
    h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
    
    vespos = vessel.position(bref)
    
    vesvpos = vessel.velocity(bref)
    
    tpos = target.position(bref)
    tpos = [tpos[0], tpos[2], tpos[1]]
    
    x_1 = pred.Toggle(vespos, vesvpos)#predicted landing location based on current position and velocity vectors in Cartesian coords
    lat = pred.latlon(x_1)[0]
    lon = pred.latlon(x_1)[1]
    
    cur_lat = vessel.flight(bref).latitude
    cur_lon = vessel.flight(bref).longitude
    
    target_lat = pred.latlon(tpos)[0]
    target_lon = pred.latlon(tpos)[1]

    #Latitudes and Longitudes of all relative positions
    lat = pred.d2r(lat)
    lon = pred.d2r(lon)
    cur_lat = pred.d2r(cur_lat)
    cur_lon = pred.d2r(cur_lon)
    target_lat = pred.d2r(target_lat)
    target_lon = pred.d2r(target_lon)
    
    smode1.insert(0, 0)

    #The following calculates the deviation of the landing position from the line connecting the vessel and the target
    t_p = pred.db2(target_lat, target_lon, lat, lon)
    p_v = pred.db2(lat, lon, cur_lat, cur_lon)
    t_v = pred.db2(target_lat, target_lon, cur_lat, cur_lon)

    vespos = [vespos[0],vespos[2],vespos[1]]
    vespos = numpy.multiply(vespos, br/numpy.linalg.norm(vespos))
    n_vec = numpy.cross(vespos, tpos)
    n_vec = numpy.divide(n_vec, numpy.linalg.norm(n_vec))
    xp = pred.r2d(numpy.arcsin(numpy.dot(n_vec, numpy.divide(x_1,numpy.linalg.norm(x_1)))))

    #Modifies steering direction to minimise the deviation calculated above
    deviation = predir*xp
    Dir_Bearing = numpy.asarray(Dir)
    Dir_Bearing[0] = 0
    Dir_Bearing = numpy.divide(Dir_Bearing, numpy.linalg.norm(Dir_Bearing))
    r_ang = pred.d2r(100*deviation)
    
    r_ang = [[1, 0, 0],[0, numpy.cos(r_ang), -numpy.sin(r_ang)],[0, numpy.sin(r_ang), numpy.cos(r_ang)]]
    r_ang = numpy.transpose(r_ang)
    Dir_Bearing = numpy.dot(r_ang, Dir_Bearing)

    new_Dir = (Dir_Bearing[0],Dir_Bearing[1], Dir_Bearing[2])
    e.insert(0,new_Dir)
    
    error_ang = SAS.vang(numpy.asarray(new_Dir), numpy.asarray(vessel.flight(vessel.surface_reference_frame).direction))
    
    if error_ang<=0.3745:
        vessel.control.throttle = 1.0
        vessel.control.rcs = False
    else:
        vessel.control.throttle = 0.0
        vessel.control.rcs = True
    lim = pred.r2d((p_v-t_v)/br)
    #condition to slightly overshoot the target to account for drag and landing maneuvers
    if lim>0.06 and lim<0.07 and (t_p-t_v)<=0:#change these conditions depending on trajectory
        vessel.control.throttle = 0.0
        vessel.control.rcs = True
        break
    
    time.sleep(0.005)

#change SAS to work relative to the surface velocity
smode1.insert(0, 1)
Dir = (0.0, -1.0, 0.0)
e.insert(0,Dir)

#Re-Entry Burn
while h<=330000:
    conn.drawing.clear()
    h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
    if h<=77000:#change this condition depending on trajectory 
        SAS.RotPitch.Kd = 0.25
        SAS.RotHeading.Kd = 0.25
        SAS.RotRoll.Kd = 0.25

        SAS.TorKp[0] = 0.45*10.0
        SAS.TorKp[1] = 0.45*10.0
        SAS.TorKp[2] = 0.45*10.0*0.25
        SAS.TorKd = numpy.multiply(SAS.TorKp, 10.0)
        if surface_speed()>=1136.0:#change this condition depending on trajectory 
            mode = 2
            Dir = conn.space_center.transform_direction(PN.Toggle(mode, target), vessel.reference_frame, vessel.surface_velocity_reference_frame)
            Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
            e.insert(0,Dir)
            vessel.control.throttle = 1.0
        else:
            vessel.control.throttle = 0.0
            break
    else:
        vessel.control.throttle = 0.0
    time.sleep(0.005)

#Landing Burn
vessel.control.set_action_group(2, True)    

t = threading.Thread(target = T.Toggle, args=(w, target_offset, 3500))#change 3rd argument depending on trajectory
t.start()

sign = 0.2
while sign>0:
    sign = vertical_vel()/numpy.abs(vertical_vel())
    time.sleep(0.01)

while (sign*h)<0:
    sign = vertical_vel()/numpy.abs(vertical_vel())
    
    h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1] - target_offset
    
    i = True

    q.insert(0,i)

    #changes the result of the Proportional Navigation module to suit different regimes during descent
    if h<70000 and h>=10000:
        mode = 0
        Dir = conn.space_center.transform_direction(PN.Toggle(mode, target), vessel.reference_frame, vessel.surface_velocity_reference_frame)
        Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
        e.insert(0,Dir)
    elif h<10000 and h>=300:
        if numpy.abs(vertical_vel())>=120.0:
            mode = 1
            Dir = conn.space_center.transform_direction(PN.Toggle(mode, target), vessel.reference_frame, vessel.surface_velocity_reference_frame)
            Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
            e.insert(0,Dir)
        if numpy.abs(vertical_vel())<120.0:
            mode = 2
            Dir = conn.space_center.transform_direction(PN.Toggle(mode, target), vessel.reference_frame, vessel.surface_velocity_reference_frame)
            Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
            e.insert(0,Dir)
    elif h<300:
        mode = 3
        Dir = conn.space_center.transform_direction(PN.Toggle(mode, target), vessel.reference_frame, vessel.surface_velocity_reference_frame)
        Dir = numpy.multiply(Dir, 1.0/numpy.linalg.norm(Dir))
        e.insert(0,Dir)
    o = True
    w.insert(0,o)

    time.sleep(0.005)

i = False
q.insert(0,i)
o = False
w.insert(0,o)
vessel.control.sas = True
t.join()
print(t.is_alive())
s.join()
print(s.is_alive())
conn.close()
