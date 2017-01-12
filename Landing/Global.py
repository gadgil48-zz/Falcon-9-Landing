import krpc
import numpy
import time

global conn
global vessel

conn = krpc.connect(name='Landing')
vessel = conn.space_center.active_vessel

#change this as required for different launches
for i in conn.space_center.vessels:
    if(i.name == 'ASDS-3'):
        target = i
