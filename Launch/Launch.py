import krpc, math
import numpy
import time
import Global
conn = Global.conn
vessel = Global.vessel
target = Global.target

#change these as desired
global target_apoapsis
global target_inclination
global ftarget_apoapsis
target_apoapsis = 262#first target orbital apoapsis
ftarget_apoapsis = 262#second target orbital apoapsis 
target_inclination = 28.6085
target_const = 900#velocity at which pitch angle should be 45 degrees from horizon

global b_ref
b_ref = vessel.orbit.body.reference_frame

global r2d
global d2r
r2d = 180.0/numpy.pi
d2r = numpy.pi/180.0

ns = 0

global mode
mode = 0

if mode==0:
    #calculation of launch azimuth

    rot = numpy.arcsin(numpy.cos(target_inclination*d2r)/numpy.cos(vessel.flight(b_ref).latitude*d2r))
    if ns == 0:
        rot = rot
    elif ns == 1:
        rot = numpy.pi-rot
    rot *= r2d
    
    print("Launch Azimuth: %f" % (rot))

    print("Desired inclination: %f" % target_inclination)

    time.sleep(10)

vessel.control.throttle = 1.0
time.sleep(5)
vessel.control.activate_next_stage()
time.sleep(5)
vessel.control.activate_next_stage()

vessel.auto_pilot.engage()

def launch_steering(check ,rot, target_const):
    while vessel.orbit.apoapsis_altitude<target_apoapsis*1000:
        v_surf_count = vessel.flight(vessel.orbit.body.reference_frame).speed
        #gravity turn
        pitch_ang = numpy.arctan(target_const/v_surf_count)*180.0/numpy.pi
        #mode 1 should never be necessary provided the landing platform is positioned in line with launch_azimuth
        if mode==1:
            vlat = vessel.flight(b_ref).latitude*numpy.pi/180.0
            vlon = vessel.flight(b_ref).longitude*numpy.pi/180.0

            tlat = target.flight(b_ref).latitude*numpy.pi/180.0
            tlon = target.flight(b_ref).longitude*numpy.pi/180.0

            rot = numpy.arctan2(numpy.sin(tlon-vlon)*numpy.cos(tlat),numpy.cos(vlat)*numpy.sin(tlat) - numpy.sin(vlat)*numpy.cos(tlat)*numpy.cos(tlon-vlon))
            rot*=r2d
        
        #MECO-1
        if check == 0:
            if v_surf_count<=2300:        
                vessel.auto_pilot.target_pitch_and_heading(pitch_ang, rot)
            else:
                vessel.control.throttle = 0.0
                break
        elif check == 1:
            vessel.auto_pilot.target_pitch_and_heading(pitch_ang, rot)
        time.sleep(0.01)

launch_steering(0, rot, target_const)
time.sleep(2)
vessel.control.rcs = True
vessel.control.set_action_group(1, True)
vessel.control.activate_next_stage()
time.sleep(2)
vessel.control.activate_next_stage()
time.sleep(2)
vessel.control.throttle = 1.0
launch_steering(1, rot, target_const+500)

vessel.auto_pilot.disengage()
import Final_stage
Final_stage.Final_stage(target_apoapsis, rot, ftarget_apoapsis)
conn.close()
