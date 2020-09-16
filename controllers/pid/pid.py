#!/usr/bin/env python3
from controller import Robot

# valores do pid
ganho_p = 0.5
ganho_i = 0.00001
ganho_d = 0.1
integral = 0
erro_antigo = 0

# valores de velocidade
# vel_giro = 2.0
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
so4 = robot.getDistanceSensor('so4')
so4.enable(timestep)
so5 = robot.getDistanceSensor('so5')
so5.enable(timestep)
so6 = robot.getDistanceSensor('so6')
so6.enable(timestep)
so7 = robot.getDistanceSensor('so7')
so7.enable(timestep)

# função de pid 
def pid(valor_atual, valor_desejado):

    global ganho_p, ganho_i, ganho_d
    global integral, erro_antigo

    # atualização do erro
    erro = valor_desejado - valor_atual
    integral = integral + erro
    diferenca_erro = erro - erro_antigo
    erro_antigo = erro
    
    # PID 
    P = ganho_p * erro 
    I = ganho_i * integral
    D = ganho_d * diferenca_erro
    
    # return P
    return P+D
    # return P+I
    # return P+I+D
        
# função de seguir parede
def segidor_parede_direita(dist_goal):
    print ('seguidor de parede (direita)')

    dist_4 = 1024 - so4.getValue()
    dist_5 = 1024 - so5.getValue()
    dist_6 = 1024 - so6.getValue()
    dist_7 = 1024 - so7.getValue()
    dist_min = min([dist_6,dist_7])           

    if dist_4 < dist_goal or dist_5 < dist_goal:
        frw.setVelocity(vel_min)
        flw.setVelocity(-vel_min)
        brw.setVelocity(vel_min)
        blw.setVelocity(-vel_min)
    else:
        power = pid(dist_min, dist_goal)
        vel_pid = vel_min + power
        
        if vel_pid > vel_max:
            vel_pid = vel_max
        if vel_pid < -vel_max:
            vel_pid = -vel_max

        frw.setVelocity(vel_pid)
        flw.setVelocity(vel_min)
        brw.setVelocity(vel_pid)
        blw.setVelocity(vel_min)

while robot.step(timestep) != -1:
    segidor_parede_direita(300)

