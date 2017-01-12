import krpc
import numpy
import time

'''Custom PID Library based on the PID Class available in the kOS mod for KSP no real need to change but look in the kOS docs for more info'''

class PID:
    def __init__(self, LastInput, LastSampleTime, ErrorSum, Kp, Ki, Kd, Setpoint, MinOutput, MaxOutput):
        self.LastInput = LastInput
        self.LastSampleTime = LastSampleTime
        self.ErrorSum = ErrorSum
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Setpoint = Setpoint
        self.MinOutput = MinOutput
        self.MaxOutput = MaxOutput
        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        self.deadband = 0

    def Update(self, sampleTime, Input):
        Error = self.Setpoint - Input
        self.PTerm = self.Kp*Error
        self.ITerm = 0
        self.DTerm = 0
        in_deadband = abs(Error) < self.deadband
        
        if(self.LastSampleTime < sampleTime):
            if not in_deadband:
                dt = sampleTime - self.LastSampleTime
                if(numpy.absolute(Error)>numpy.power(10.0, -6.0)):
                    if(self.Ki>0 or self.Ki<0):
                        self.ITerm = (self.ErrorSum + Error*dt)*self.Ki

                    ChangeRate = (Input-self.LastInput)/dt

                    if(self.Kd>0 or self.Kd<0):
                        self.DTerm = -ChangeRate*self.Kd

        Output = self.PTerm + self.ITerm + self.DTerm

        if(Output>self.MaxOutput):
            Output = self.MaxOutput
            #Prevent Integral Wind-up

            if((self.Ki<0 or self.Ki>0) and self.LastSampleTime<sampleTime):
                self.ITerm = Output - min(self.PTerm+self.DTerm, self.MaxOutput)


        elif(Output<self.MinOutput):
            Output = self.MinOutput

            if((self.Ki>0 or self.Ki<0) and self.LastSampleTime<sampleTime):
                self.ITerm = Output - max(self.PTerm + self.DTerm, self.MinOutput)


        self.LastSampleTime = sampleTime
        self.LastInput = Input

        if(self.Ki<0 or self.Ki>0):
            self.ErrorSum = self.ITerm/self.Ki
        else:
            self.ErrorSum = 0

        return Output

    def Reset(self):
        self.ErrorSum = 0
        self.ITerm = 0
        self.LastSampleTime = self.MaxOutput

