import krpc
import numpy
import time
import threading
import Global
import PIDLoop
conn = Global.conn
vessel = Global.vessel
canvas = conn.ui.stock_canvas

screen_size = canvas.rect_transform.size


panel = canvas.add_panel()

constant = 0.0

def Toggle(in_r, target):
    mode = in_r

    #tedious transformations
    target_position = target.position(target.orbit.body.reference_frame)
    target_position = numpy.asarray(conn.space_center.transform_direction(target_position, target.orbit.body.reference_frame, target.surface_reference_frame))
    #custom value for target position to ensure a trajectory that intercepts the target tail on instead of severe angle
    target_position[0] += 0.5*vessel.flight(vessel.orbit.body.reference_frame).mean_altitude
    target_position = (target_position[0], target_position[1], target_position[2])
    target_position = conn.space_center.transform_direction(target_position, target.surface_reference_frame, target.orbit.body.reference_frame)
    target_position = numpy.asarray(target_position)

    vessel_position = numpy.asarray(vessel.position(vessel.orbit.body.reference_frame))
    target_position = numpy.add(target_position, numpy.multiply(vessel_position, -1.0))
    target_position = (target_position[0], target_position[1], target_position[2])
    target_position = conn.space_center.transform_direction(target_position, target.orbit.body.reference_frame, vessel.reference_frame)
    target_position = numpy.asarray(target_position)

        
    relative_velocity = vessel.velocity(vessel.orbit.body.reference_frame)

    relative_velocity = conn.space_center.transform_direction(relative_velocity, vessel.orbit.body.reference_frame, vessel.reference_frame)

    relative_velocity = numpy.asarray(relative_velocity)

    relative_velocity = numpy.multiply(relative_velocity, -1.0)

    Omega = numpy.cross(target_position, relative_velocity)/numpy.vdot(target_position, target_position)

    A_Vec = numpy.multiply(numpy.cross(relative_velocity, Omega), 5.0)
    
    travelling = vessel.velocity(vessel.orbit.body.reference_frame)
    travelling = numpy.multiply(travelling, 1.0/numpy.linalg.norm(travelling))
    travelling = conn.space_center.transform_direction(travelling, vessel.orbit.body.reference_frame, vessel.surface_reference_frame)
    travelling = numpy.multiply(travelling, -1.0)
    error_ang = 180.0*numpy.arctan(numpy.sqrt(travelling[1]*travelling[1] + travelling[2]*travelling[2])/travelling[0])/numpy.pi
    #various modes that can be triggered from Main.py
    normrv = numpy.linalg.norm(relative_velocity)
    engine_off = -35.0*((normrv-100.0)/500.0)**1.5
    engine_on = 15.0*(-numpy.tanh((normrv-200.0)/20.0)-0.9)
    if(mode == 0):
        Target_Vec = numpy.add(numpy.multiply(A_Vec, -50.0), relative_velocity)
    elif(mode == 1):
        Target_Vec = numpy.add(numpy.multiply(A_Vec, engine_off), relative_velocity)
    elif(mode == 2):
        Target_Vec = numpy.add(numpy.multiply(A_Vec, engine_on), relative_velocity)
    elif(mode == 3):
        Target_Vec = relative_velocity
    elif(mode == 4):
        Target_Vec = numpy.add(numpy.multiply(A_Vec, 5.0), relative_velocity)
    
    #actual execution of proportional navigation
    #for more info see https://en.wikipedia.org/wiki/Proportional_navigation
    target_position = (target_position[0], target_position[1], target_position[2])
    A_Vec = (A_Vec[0], A_Vec[1], A_Vec[2])
    relative_velocity = (A_Vec, relative_velocity[1], relative_velocity[2])
    Target_Vec = (Target_Vec[0], Target_Vec[1], Target_Vec[2])
    
    return Target_Vec
