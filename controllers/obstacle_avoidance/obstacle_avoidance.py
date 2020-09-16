#!/usr/bin/env python3
from controller import Robot, GPS, Compass
from math import atan2, degrees

# valores de velocidade
vel_min = 4.0
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

# inicialização dos sensores
so0 = robot.getDistanceSensor('so0')
so0.enable(timestep)
so1 = robot.getDistanceSensor('so1')
so1.enable(timestep)
so2 = robot.getDistanceSensor('so2')
so2.enable(timestep)
so3 = robot.getDistanceSensor('so3')
so3.enable(timestep)
so4 = robot.getDistanceSensor('so4')
so4.enable(timestep)
so5 = robot.getDistanceSensor('so5')
so5.enable(timestep)
so6 = robot.getDistanceSensor('so6')
so6.enable(timestep)
so7 = robot.getDistanceSensor('so7')
so7.enable(timestep)
gps = GPS('gps')
gps.enable(timestep)
compass = Compass('compass')
compass.enable(timestep)

def girar(orientacao):
    print ('girar ' + orientacao)
    if(orientacao is 'esquerda'):
        frw.setVelocity(vel_min)
        flw.setVelocity(-vel_min)
        brw.setVelocity(vel_min)
        blw.setVelocity(-vel_min)
    elif(orientacao is 'direita'):
        frw.setVelocity(-vel_min)
        flw.setVelocity(vel_min)
        brw.setVelocity(-vel_min)
        blw.setVelocity(vel_min)
        
def curva(orientacao):
    print ('curva ' + orientacao)
    if(orientacao is 'esquerda'):
        frw.setVelocity(vel_max)
        flw.setVelocity(vel_min)
        brw.setVelocity(vel_max)
        blw.setVelocity(vel_min)
    elif(orientacao is 'direita'):
        frw.setVelocity(vel_min)
        flw.setVelocity(vel_max)
        brw.setVelocity(vel_min)
        blw.setVelocity(vel_max)
        
def em_frente():
    print ('em frente')
    frw.setVelocity(vel_max)
    flw.setVelocity(vel_max)
    brw.setVelocity(vel_max)
    blw.setVelocity(vel_max)

def leituraSonares():
    dist_0 = 1024 - so0.getValue()
    dist_1 = 1024 - so1.getValue()
    dist_2 = 1024 - so2.getValue()
    dist_3 = 1024 - so3.getValue()
    dist_4 = 1024 - so4.getValue()
    dist_5 = 1024 - so5.getValue()
    dist_6 = 1024 - so6.getValue()
    dist_7 = 1024 - so7.getValue()
    distancias = [dist_0, dist_1, dist_2, dist_3,
                  dist_4, dist_5, dist_6, dist_7]
    # 0, 1, 2, 3 = ESQ
    # 4, 5, 6, 7 = DIR
    dist_min_lados = [min(distancias[0:4]),min(distancias[4:8])]
    return distancias, dist_min_lados

def bug_tangente():

    x_objetivo = 0
    y_objetivo = 0
    z_objetivo = 0
    
    x, y, z = gps.getValues()
    x_mag, y_mag, z_mag = compass.getValues()
    yaw = atan2(z_mag,x_mag)   
    
    angulo_objetivo = atan2(x-x_objetivo,z-z_objetivo)
    diferenca = yaw - angulo_objetivo
    
    distancias, dist_min_lados = leituraSonares()
    if(min(distancias) > 100):
        if(degrees(diferenca) <= -5):
            girar('esquerda')
        elif(degrees(diferenca) >= 5):
            girar('direita')
        else:
            em_frente()
    else:
        if dist_min_lados[0] < dist_min_lados[1]:
            curva('direita')
        else:
            curva('esquerda')

while robot.step(timestep) != -1:
    bug_tangente()