#!/usr/bin/env python
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
import paho.mqtt.publish as publish
import numpy as np
import meArm

Broker = "192.168.0.17"
sub_topic = "dev/test"    # receive messages on this topic
pub_topic = "dev/test"    # send messages to this topic

arm = meArm.meArm()
arm.begin()

def on_message(client, userdata, msg):
    x = 0
    z = 0
    num_fingers = 0
    area = 0

    coord = bytearray(msg.payload)
    x_byte = bytearray([coord[0],coord[1]])
    z_byte = bytearray([coord[2],coord[3]])
    fingers_byte = bytearray([coord[4]])


    x = int(x)
    z = int(z)
    num_fingers = int(num_fingers)
    area = int(area)
    
    x = int.from_bytes(x_byte, 'big')
    z = int.from_bytes(z_byte, 'big')
    num_fingers = int.from_bytes(fingers_byte, 'big')
    
    print("x: ",x)
    print("z: ",z)
    print("number of pointed fingers: ", num_fingers)
    
    grid_x = -150+((x/480)*300)  # rounds x and y coordinates to nearest 10% value 
    grid_z = 50+((z/320)*150)

    
    arm.goDirectlyTo(grid_x,50,grid_z)

    
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(sub_topic)


def on_publish(mosq, obj, mid):
    print("mid: " + str(mid))
    
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(Broker, 1883, 60)
client.loop_forever()

root.mainloop()
