#!/usr/bin/env python3
from controller import Robot, GPS, Compass
from math import atan2, degrees, sqrt, pow
from numpy import ones, transpose

vel = 4

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

# inicialização do GPS
gps = GPS('gps')
gps.enable(timestep)

# inicialização da bússola
compass = Compass('compass')
compass.enable(timestep)

# função para parar
def parar():
    print ('parar')
    frw.setVelocity(0)
    flw.setVelocity(0)
    brw.setVelocity(0)
    blw.setVelocity(0)

# função para girar
def girar(orientacao):
    print ('girar ' + orientacao)
    if(orientacao is 'esquerda'):
        frw.setVelocity(vel)
        flw.setVelocity(-vel)
        brw.setVelocity(vel)
        blw.setVelocity(-vel)
    elif(orientacao is 'direita'):
        frw.setVelocity(-vel)
        flw.setVelocity(vel)
        brw.setVelocity(-vel)
        blw.setVelocity(vel)

# função para seguir em frente
def em_frente():
    print ('em frente')
    frw.setVelocity(vel)
    flw.setVelocity(vel)
    brw.setVelocity(vel)
    blw.setVelocity(vel)
    
##################################################

parar()

# objetivo 
cel_objetivo_x = 10
cel_objetivo_z = 8

# inicialização do mapa
mapa = ones((12, 12))
mapa[1:11,1:11] = 0
mapa[ 4, 1] = 1
mapa[ 1, 3] = 1
mapa[ 8, 3] = 1
mapa[ 3, 4] = 1
mapa[10, 4] = 1
mapa[ 6, 5] = 1
mapa[ 4, 7] = 1
mapa[ 8, 7] = 1
mapa[ 8, 9] = 1
mapa[ 1,10] = 1
mapa[cel_objetivo_x,cel_objetivo_z] = 2
print(transpose(mapa))

##################################################

def grafo(cel_x,cel_z):
    conectividade_pos = []

    # esquerda
    if(mapa[cel_x-1,cel_z] != 1):
        conectividade_pos.append([cel_x-1,cel_z])

    # direita
    if(mapa[cel_x+1,cel_z] != 1):
        conectividade_pos.append([cel_x+1,cel_z])    

    # topo
    if(mapa[cel_x,cel_z-1] != 1):
        conectividade_pos.append([cel_x,cel_z-1])

    # base
    if(mapa[cel_x,cel_z+1] != 1):
        conectividade_pos.append([cel_x,cel_z+1])
        
    return conectividade_pos

def heuristica(grafo):
    conectividade_val = []
    
    for no in grafo:
        val = sqrt(pow(no[0]-cel_objetivo_x,2)
                 + pow(no[1]-cel_objetivo_z,2))
        conectividade_val.append(val)
    
    return conectividade_val                 
    
            
plano = []
cel_atual_x = 1
cel_atual_z = 1
while(mapa[cel_atual_x,cel_atual_z] != 2):

    g = grafo(cel_atual_x,cel_atual_z)
    h = heuristica(g)
    
    min_value = min(h)
    min_index = h.index(min_value)
    min_cell = g[min_index]
    
    plano.append(min_cell)
    cel_atual_x = min_cell[0]
    cel_atual_z = min_cell[1]

    robot.step(timestep)
    print(transpose(mapa))
    print(plano)

# Execução do planejamento
while robot.step(timestep) != -1:
    if(len(plano) == 0):
        parar()
        print("cheguei")
        break
    else:
        objetivo_x = plano[0][0] + 0.5
        objetivo_z = plano[0][1] + 0.5
        
        x, y, z = gps.getValues()        
        x_mag, y_mag, z_mag = compass.getValues()
        
        yaw = atan2(z_mag,x_mag)       
        angulo_objetivo = atan2(x-objetivo_x,z-objetivo_z)

        diferenca = yaw - angulo_objetivo        
        dist_objetivo = sqrt(pow(x-objetivo_x,2) + pow(z-objetivo_z,2))

        if(dist_objetivo < 0.1):
            del plano[0]
        else:
            if(degrees(diferenca) <= -2):
                girar('esquerda')
            elif(degrees(diferenca) >= 2):
                girar('direita')
            else:
                em_frente()