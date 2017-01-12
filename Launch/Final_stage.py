import krpc, math
import numpy as np
import time
import Global
import PIDLoop
import threading
conn = Global.conn
vessel = Global.vessel
target = Global.target

vs = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
ap = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
pe = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')

vessel.auto_pilot.auto_tune = True
vessel.auto_pilot.engage()


def Final_stage(ap_setpoint, heading, ftarget_apoapsis):
    
    setpoint = ap_setpoint*1000
    Kp =1.0
    Ki =0.1
    Kd =0.0
    PitchPID = PIDLoop.PID(0.0, time.time(), 0.0, Kp, Ki, Kd, setpoint, -60, 60)
    vessel.control.throttle = 1.0
    
    start_time = time.time()

    '''first stage of orbital insertion before reaching apoapsis
    this aims to keep the orbit apoapsis at the setpoint specified'''
    while vs()>0:
        cur_time = time.time() - start_time
        PitchPID.Setpoint = setpoint
        PitchPID.deadband = 100
        Tu = 25.8
        Ku = 0.015
        PitchPID.Kp = 0.2*Ku
        PitchPID.Ki = 2.0*PitchPID.Kp/Tu
        PitchPID.Kd = PitchPID.Kp*Tu/3.0
            
        pitch = PitchPID.Update(time.time(), ap())
        vessel.auto_pilot.target_pitch_and_heading(pitch, heading)
        time.sleep(0.01)

    '''second stage of orbital insertion after reaching apoapsis
    this aims to keep the vertical velocity as close to zero(actually -0.2) as possible'''
    while pe()<142000:
        cur_time = time.time() - start_time
        PitchPID.Setpoint = -0.2
        PitchPID.deadband = 0.5
        Tu = 17.0
        Ku = 100.0*0.003
        PitchPID.Kp = 0.2*Ku
        PitchPID.Ki = 2.0*PitchPID.Kp/Tu
        PitchPID.Kd = PitchPID.Kp*Tu/3.0

        pitch = PitchPID.Update(time.time(), vs())
        vessel.auto_pilot.target_pitch_and_heading(pitch, heading)
        time.sleep(0.01)
    '''third stage(optional) of oribtal insertion after reaching apoapsis
    this aims point the craft at the horizon and burn until required final
    apoapsis value is reached'''
    while ap()<ftarget_apoapsis*1000:
        vessel.auto_pilot.target_pitch_and_heading(0, heading)
        time.sleep(0.01)
    vessel.control.throttle = 0.0
