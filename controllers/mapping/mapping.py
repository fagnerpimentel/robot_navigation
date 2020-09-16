#!/usr/bin/env python3
from controller import Robot, GPS, Compass
from math import atan2, degrees, pi, sqrt, pow, floor
from numpy import zeros
from random import choice

# mapa
mapa = zeros((10, 10))

# valores de velocidade
vel_min = 0.5
vel_max = 6.4

# inicialização do robô
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# inicialização dos atuadores
frw = robot.getMotor('front right wheel')
frw.setPosition(float('inf'))
flw = robot.getMotor('front left wheel')
flw.setPosition(float('inf'))
brw = robot.getMotor('back right wheel')
brw.setPosition(float('inf'))
blw = robot.getMotor('back left wheel')
blw.setPosition(float('inf'))

# inicialização dos sonareas
so4 = robot.getDistanceSensor('so4')
so4.enable(timestep)
so5 = robot.getDistanceSensor('so5')
so5.enable(timestep)

# inicialização do GPS
gps = GPS('gps')
gps.enable(timestep)

# inicialização da bússola
compass = Compass('compass')
compass.enable(timestep)

def leituraSonares():
    dist_4 = 1024 - so4.getValue()
    dist_5 = 1024 - so5.getValue()
    return min(dist_4, dist_5)

def girar(orientacao):
    print ('girar ' + orientacao)
    
    if(orientacao is 'esquerda'):
        ang = -90
    elif(orientacao is 'direita'):
        ang = 90
    else:
        ang = 0    

    x_mag, y_mag, z_mag = compass.getValues()
    yaw_1 = atan2(z_mag,x_mag)       
    while True:
        x_mag, y_mag, z_mag = compass.getValues()
        yaw_2 = atan2(z_mag,x_mag)
        
        yaw = yaw_1-yaw_2
        while yaw > pi:
            yaw -= 2*pi
        while yaw < -pi:
            yaw += 2*pi
        
        if ang > degrees(yaw) + 0.5:
            frw.setVelocity(-vel_min)
            flw.setVelocity(vel_min)
            brw.setVelocity(-vel_min)
            blw.setVelocity(vel_min)
        elif ang < degrees(yaw) - 0.5:
            frw.setVelocity(vel_min)
            flw.setVelocity(-vel_min)
            brw.setVelocity(vel_min)
            blw.setVelocity(-vel_min)
        else:
            break
        robot.step(timestep)

def em_frente():
    print ('em frente')

    dist = 1

    x_1, y_1, z_1 = gps.getValues()
    while True:
        x_2, y_2, z_2 = gps.getValues()
        
        dif = sqrt(pow(x_2-x_1,2)+pow(z_2-z_1,2))

        frw.setVelocity(vel_max)
        flw.setVelocity(vel_max)
        brw.setVelocity(vel_max)
        blw.setVelocity(vel_max)
        
        if(dif > dist):
            break
        if (leituraSonares() < 70):
            break
        robot.step(timestep)
        
def atualiza_mapa():
    x, y, z = gps.getValues()
    mapa[floor(z),floor(x)] = 1    

while robot.step(timestep) != -1:
    atualiza_mapa()
    print(mapa)
    
    # if(leituraSonares() < 70):
        # lista_orientacao = ['direita','esquerda']
        # orientacao = choice(lista_orientacao)
        # girar(orientacao)
    # else:
        # em_frente()  

    lista_orientacao = ['frente','direita','esquerda']
    orientacao = choice(lista_orientacao)
    if(orientacao is 'frente'):
        em_frente()
    else:
        girar(orientacao)
    
