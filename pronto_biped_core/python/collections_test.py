#!/usr/bin/python

import lcm
from lcm import LCM
import math

from vs.object_t import object_t
from vs.object_collection_t import object_collection_t




lc = lcm.LCM()
print "started"


m = object_collection_t()


for i in range(0,5):
    print i
    msg2 = object_t();
    msg2.id= i-1;
    msg2.x= i;
    msg2.y= 2;
    msg2.z= 0;
    msg2.qw= 1;
    msg2.qx= 0;
    msg2.qy= 0;
    msg2.qz= 0;
    m.objects.append(msg2)
    lc.publish("XXDSFDS", msg2.encode())

m.id =1;
m.type =5;
m.name ="Name";
m.reset =True
m.nobjects = len(m.objects)

lc.publish("OBJECT_COLLECTION", m.encode())
