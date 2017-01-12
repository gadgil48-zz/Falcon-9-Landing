import krpc
import numpy
import time
import threading
import Global
import PIDLoop
conn = Global.conn
vessel = Global.vessel

altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'surface_altitude')
vertical_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')

h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]

#Gain values for throttle PID
ThrottKp = 2.0
ThrottKi = 2.4
ThrottKd = 0.0
ThrottOut = 0.0

#scaling to account for air resistance
gravscale = 0.2

Throttle_PID_setpoint = 0.4

Throttle_PID = PIDLoop.PID(0.0, time.time(), 0.0, ThrottKp, ThrottKi, ThrottKd, Throttle_PID_setpoint, 0.05, 1.0)

def Toggle(in_w, target_offset,h_lim):
    #similar to Steering_Logic
    while True:
        i = in_w[0]

        h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1] - 5.0 - target_offset

        sign = vertical_vel()/numpy.abs(vertical_vel())
        
        if(i==True):
            grav = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)

            #burn countdown calculation
            finalv = sign*numpy.sqrt(vertical_vel()*vertical_vel() + 2.0*h*(gravscale*grav))

            t2i = (finalv-vertical_vel())/(gravscale*grav)

            acc = vessel.available_thrust/vessel.mass

            t2b = finalv/(-acc)

            bc = t2b+t2i

            #calculation of throttle value
            if bc>-0.75 and h<h_lim:
                ThrottOut = Throttle_PID.Update(time.time(), -bc)
            else:
                ThrottOut = 0.0

            if h<350.0:
                vessel.control.gear = True
            
            vessel.control.throttle = ThrottOut
            time.sleep(0.01)
        elif(i==False):
            vessel.control.throttle = 0.0
            if((sign*h)>0):
                break
    
