import krpc
import time, math
import numpy
import threading
import Global
import PIDLoop

conn = Global.conn
vessel = Global.vessel

altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'surface_altitude')
vertical_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')

'''The following is a custom steering code there is no need to tweak it but some details are provided below'''

Seek_Dir = numpy.array((0.0,numpy.pi*0.5,0.0))
Current_Dir = numpy.array((0.0,0.0,0.0))
Current_Rot_Vel = (0.0,0.0,0.0)

RotKp = 1.0
RotKi = 0.1
RotKd = 7.0

Max = numpy.array((0.0,0.0,0.0))
Max[0] = 10.0*numpy.pi/180.0
Max[1] = 10.0*numpy.pi/180.0
Max[2] = 10.0*numpy.pi/180.0
Min = numpy.multiply(Max, -1.0)

#Three PID Loops which output desired angular velocity
RotPitch = PIDLoop.PID(0.0, time.time(), 0.0, RotKp, RotKi, RotKd, 0.0, Min[0], Max[0])
RotHeading = PIDLoop.PID(0.0, time.time(), 0.0, RotKp, RotKi, RotKd, 0.0, Min[1], Max[1])
RotRoll = PIDLoop.PID(0.0, time.time(), 0.0, RotKp, RotKi, RotKd, 0.0, Min[2], Max[2])

#Basis vectors
Fore = (0,1,0)
Top = (0,0,-1)
Star = (1,0,0)

#Gain values for 2nd set of PID Loops
TorTu = 8.0
TorKu = 5.0
TorKi = numpy.array((0.0,0.0,0.0))
TorKp = numpy.array((0.0,0.0,0.0))
TorKd = numpy.array((0.0,0.0,0.0))
TorKp[0] = 0.45*TorKu
TorKp[1] = 0.45*TorKu
TorKp[2] = 0.45*TorKu*0.25
TorKd = numpy.multiply(TorKp, 10.0)


def r2d(x):
    return x*180.0/numpy.pi

def d2r(x):
    return x*numpy.pi/180.0

def simp(x):
    return round(x,2)

def vang(v1, v2):
    return numpy.arccos(numpy.vdot(v1,v2))

def vexclude(v1, v2):
    n = numpy.cross(numpy.cross(v1, v2)/numpy.linalg.norm(numpy.cross(v1, v2)), v1)
    return n/numpy.linalg.norm(n)


h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]

Seek_Dir1 = (0.0,1.0,0.0)

#Main function which is run in the background
def Toggle(in_q, in_e, in_mode):
    while True:
        i = in_q[0]
        sign = vertical_vel()/numpy.abs(vertical_vel())

        h = altitude() + (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
        #inputted desired direction
        Seek_Dir1 = in_e[0]
        #inputted mode
        mode = in_mode[0]
        
        if(i==True):
            vessel.control.sas = False
    
            if mode == 1:       
                Seek_Dir1 = conn.space_center.transform_direction(Seek_Dir1, vessel.surface_velocity_reference_frame, vessel.surface_reference_frame)
                TorKu = 5.0
                TorKi = numpy.array((0.0,0.0,0.0))
                TorKp = numpy.array((0.0,0.0,0.0))
                TorKd = numpy.array((0.0,0.0,0.0))
                TorKp[0] = 0.45*TorKu
                TorKp[1] = 0.45*TorKu
                TorKp[2] = 0.45*TorKu*0.25
                TorKd = numpy.multiply(TorKp, 10.0)
            else:
                TorKu = 5.0
                TorKi = numpy.array((0.0,0.0,0.0))
                TorKp = numpy.array((0.0,0.0,0.0))
                TorKd = numpy.array((0.0,0.0,0.0))
                TorKp[0] = 0.45*TorKu
                TorKp[1] = 0.45*TorKu
                TorKp[2] = 0.45*TorKu*0.25*40
                TorKd = numpy.multiply(TorKp, 10.0)
                
            Seek_Dir[0] = 0.5*numpy.pi - numpy.arccos(Seek_Dir1[0])
            Seek_Dir[1] = numpy.arctan2(Seek_Dir1[2],Seek_Dir1[1])
            if((Seek_Dir[1]*180.0/numpy.pi)<0.0):
                Seek_Dir[1] += 2.0*numpy.pi
            else:
                Seek_Dir[1] += 0.0
            Seek_Dir[2] = 0.0
            
            Seek_Dir1_Top = numpy.array((0.0,0.0,0.0))
            Seek_Dir1_Top[0] = numpy.cos(-Seek_Dir[0])
            if((Seek_Dir[0]*180.0/numpy.pi)<=0.0):
                Seek_Dir1_Top[1] = numpy.sin(Seek_Dir[0])*numpy.cos(Seek_Dir[1])
                Seek_Dir1_Top[2] = numpy.sin(Seek_Dir[0])*numpy.sin(Seek_Dir[1])
            else:
                Seek_Dir1_Top[1] = -numpy.sin(Seek_Dir[0])*numpy.cos(Seek_Dir[1])
                Seek_Dir1_Top[2] = -numpy.sin(Seek_Dir[0])*numpy.sin(Seek_Dir[1])
            
            #transformed direction
            Seek_Dir1 = conn.space_center.transform_direction(Seek_Dir1, vessel.surface_reference_frame, vessel.reference_frame)
            Seek_Dir1_Top = conn.space_center.transform_direction(Seek_Dir1_Top, vessel.surface_reference_frame, vessel.reference_frame)
            
            
            Current_DirMag = vang(Fore, Seek_Dir1)

            Current_Dir = numpy.array((0.0,0.0,0.0))
            Current_Dir[0] = vang(Fore, vexclude(Star, Seek_Dir1))*180.0/numpy.pi
            if(vang(Top, vexclude(Star, Seek_Dir1))*180.0/numpy.pi > 90):
                Current_Dir[0] *= -1.0

            Current_Dir[1] = vang(Fore, vexclude(Top, Seek_Dir1))*180.0/numpy.pi
            if(vang(Star, vexclude(Top, Seek_Dir1))*180.0/numpy.pi > 90):
                Current_Dir[1] *= -1.0
            
            Current_Dir[2] = vang(Top, vexclude(Fore, Seek_Dir1_Top))*180.0/numpy.pi
            if(vang(Star, vexclude(Fore, Seek_Dir1_Top))*180.0/numpy.pi > 90.0):
                Current_Dir[2] *= -1.0
                
            #transformed Current direction
            Current_Dir[0] *= (numpy.pi/180.0)
            Current_Dir[1] *= (numpy.pi/180.0)
            Current_Dir[2] *= (numpy.pi/180.0)
            
            #Current rotational velocity
            Current_Rot_Vel = vessel.angular_velocity(vessel.surface_reference_frame)
            
            Current_Rot_Vel = conn.space_center.transform_direction(Current_Rot_Vel, vessel.surface_reference_frame, vessel.reference_frame)
            Current_Rot_Vel = numpy.asarray(Current_Rot_Vel)
            
            dum = Current_Rot_Vel[1]
            Current_Rot_Vel[1] = Current_Rot_Vel[2] 
            Current_Rot_Vel[2] = dum

            Current_Rot_Vel = numpy.multiply(Current_Rot_Vel, -1.0)
            
            #desired rotational velocity
            Out_Pitch_Vel = RotPitch.Update(time.time(), -Current_Dir[0])
            Out_Heading_Vel = RotHeading.Update(time.time(), -Current_Dir[1])
            
            Out_Roll_Vel = 0.0

            Torque_Pitch = PIDLoop.PID(0.0, 0.01, 0.0, TorKp[0], TorKi[0], TorKd[0], Out_Pitch_Vel, -4.0, 4.0)
            Torque_Heading = PIDLoop.PID(0.0, 0.01, 0.0, TorKp[1], TorKi[1], TorKd[1], Out_Heading_Vel, -4.0, 4.0)
            Torque_Roll = PIDLoop.PID(0.0, 0.01, 0.0, TorKp[2], TorKi[2], TorKd[2], Out_Roll_Vel, -4.0, 4.0)
            
            #Commands to send
            Command_Pitch = Torque_Pitch.Update(time.time(), Current_Rot_Vel[0])
            Command_Heading = Torque_Heading.Update(time.time(), Current_Rot_Vel[1])
            Command_Roll = Torque_Roll.Update(time.time(), Current_Rot_Vel[2])

            Current_Dir = numpy.multiply(Current_Dir, 180.0/numpy.pi)
            
            vessel.control.roll = Command_Roll
            vessel.control.pitch = Command_Pitch
            vessel.control.yaw = Command_Heading
                        
            time.sleep(0.005)
        elif(i==False):
            #exit code
            vessel.control.pitch = 0.0
            vessel.control.yaw = 0.0
            vessel.control.roll = 0.0
            vessel.control.sas = True
            if((sign*h)>0) and vessel.flight(vessel.orbit.body.reference_frame).mean_altitude < 70000:
                break
            time.sleep(0.01)
            
