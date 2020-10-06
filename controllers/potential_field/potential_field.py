#!/usr/bin/env python3
from controller import Robot, GPS, Compass
from math import cos, sin, atan2, degrees, sqrt, pow, hypot

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

##################################################

# função para parar
def parar():
    print ('parar')
    frw.setVelocity(0)
    flw.setVelocity(0)
    brw.setVelocity(0)
    blw.setVelocity(0)

# função para girar
def girar(orientacao, vel=4):
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
def em_frente(vel=4):
    print ('em frente')
    frw.setVelocity(vel)
    flw.setVelocity(vel)
    brw.setVelocity(vel)
    blw.setVelocity(vel)

# distância euclidiana
def euclidiana(x1,z1,x2,z2):
    return sqrt( pow(x1-x2,2) + pow(z1-z2,2) )

##################################################

Katt = 10.0  # Ganho potencial atrativo
Krep = 200.0  # Ganho potencial repulsivo

def calc_campo_potencial(sx, sz, gx, gz, ox, oz, rr):
    u_atrativo = calc_potencial_atrativo(sx, sz, gx, gz)
    u_repulsivo = calc_potencial_repulsivo(sx, sz, ox, oz, rr)
    u_resultante = soma_vetorial(u_atrativo,u_repulsivo)
    
    print('atrativo:', u_atrativo)
    print('repulsivo:', u_repulsivo)
    print('resultante:', u_resultante)

    return u_resultante

def calc_potencial_atrativo(sx, sz, gx, gz):
   
    mag = 0.5 * Katt * euclidiana(sx,sz,gx,gz)
    dir = atan2(sx-gx,sz-gz)
    
    return (mag, dir)
    
def calc_potencial_repulsivo(sx, sz, ox, oz, rr):   
    vetor_repulsivo = [0,0]
    for i in range(0,len(ox)):
        vetor_aux = [0,0]
        dq = euclidiana(sx,sz,ox[i],oz[i])
        if dq <= rr:
            vetor_aux[0] = 0.5 * Krep * pow((1.0/dq - 1.0/rr),2)
        else:
            vetor_aux[0] = 0
        vetor_aux[1] = atan2(sz-oz[i],sx-ox[i])

        vetor_repulsivo = soma_vetorial(vetor_repulsivo, vetor_aux)
        
    return vetor_repulsivo
    
def soma_vetorial(v1,v2):

    # decompõe vetor 1
    vx1 = v1[0] * sin(v1[1])
    vz1 = v1[0] * cos(v1[1])

    # decompõe vetor 2
    vx2 = v2[0] * sin(v2[1])
    vz2 = v2[0] * cos(v2[1])

    # soma das decomposições
    vx3 = vx1 + vx2
    vz3 = vz1 + vz2

    # vetor resultante
    mag = euclidiana(0,0,vx3,vz3)
    dir = atan2(vx3,vz3)
    
    return (mag, dir)

##################################################
    
# objetivo 
objetivo_x = 10.5
objetivo_z = 10.5

# obstáculos
obstaculo_x = [8,3,6,4,8,8]
obstaculo_z = [3,4,5,7,7,9]

rr = 2

while robot.step(timestep) != -1:

    atual_x, atual_y, atual_z = gps.getValues()        
    x_mag, y_mag, z_mag = compass.getValues()
    yaw = atan2(z_mag,x_mag)       
    
    dist_objetivo = sqrt(pow(atual_x-objetivo_x,2) + pow(atual_z-objetivo_z,2))
    angulo_objetivo = atan2(atual_x-objetivo_x,atual_z-objetivo_z)
    
    vetor_pf = calc_campo_potencial(
        atual_x,atual_z,
        objetivo_x,objetivo_z,
        obstaculo_x,obstaculo_z,
        rr)

    if(vetor_pf[0] < 0.01):
        parar()
        print("cheguei")
        break
    else:
        diferenca = yaw - vetor_pf[1]
        if(degrees(diferenca) <= -2):
            girar('esquerda')
        elif(degrees(diferenca) >= 2):
            girar('direita')
        else:            
            vel = vetor_pf[0]
            if(vel > 6.4):
                vel = 6.4
            em_frente(vel)

        
    