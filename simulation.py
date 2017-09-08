from __future__ import division
import math
import time
import os
import pygame, sys
from pygame.locals import *
import time
import random
img = pygame.image.load('/Users/MAC/Desktop/UAV/copter_side.bmp') # Path to the side view of the copter

def rot_center(image, rect, angle):
    """rotate an image while keeping its center"""
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = rot_image.get_rect(center=rect.center)
    return rot_image,rot_rect


def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

class PID:

      def __init__(self, p_gain, i_gain, d_gain):
          self.last_error = 0.0
          self.p_gain = p_gain
          self.i_gain = i_gain
          self.d_gain = d_gain
          self.i_error = 0.0
          self.max = 400

      def updatePID(self, p_gain, i_gain, d_gain):
          self.p_gain = p_gain
          self.i_gain = i_gain
          self.d_gain = d_gain


      def Compute(self, cur, target, dt, err = 0):

          error = target - cur
          if err != 0:
              error = err

          p_error = error


          self.i_error += (error + self.last_error) * dt
          i_error = self.i_error


          d_error = (error - self.last_error) / dt


          p_output = self.p_gain * p_error
          i_output = self.i_gain * i_error
          d_output = self.d_gain * d_error


          self.last_error = error
          out = p_output+ i_output+ d_output
          if out > self.max:
              out=self.max
          if out < (-1 * self.max):
              out = (-1 * self.max)
          return int(out)


def ang_dis(tor,moi,theta,dt):
    ang_acc = tor/moi
    theta = math.radians(theta)
    theta = theta + 1/2*ang_acc*dt**2
    return math.degrees(theta)

r_pid = PID(1,1,1)
p_pid = PID(1,1,1)
y_pid = PID(1,1,1)

start_pitch = 50
start_roll = 50
start_yaw = 0
max_pwm = 2000
min_pwm = 1000
MOI = 0.07
max_thrust = 0.8 #in kg per rotor
thrust_ini = 1400 #PWM
arm_length = 0.05 #in meters
dt = 0.2
e_roll = 0
e_pitch = 0
e_yaw = 0


x = time.time()


pygame.init()
wid = 640
ht = 480
DISPLAY=pygame.display.set_mode((wid,ht),0,32)
pygame.display.set_caption("Roll/Pitch PID Stabilization simulation")
black=(0,0,0)

DISPLAY.fill(black)
pygame.display.flip()


oldRect = img.get_rect(center=(300,228))
DISPLAY.blit(img, oldRect)

pygame.font.init()

myfont = pygame.font.SysFont('Comic Sans MS', 30)

ctr = 0
dt2 = []
roll_angles = []

while True:
    roll = r_pid.Compute(start_roll,e_roll,dt)
    pitch = p_pid.Compute(start_pitch,e_pitch,dt)
    rnn = random.randint(1,10)      #Random noise in the system.

    M1_PWM = thrust_ini - roll + rnn
    M2_PWM = thrust_ini + roll + rnn
    M3_PWM = thrust_ini + roll + rnn
    M4_PWM = thrust_ini - roll + rnn

    #print valmap(M3_PWM,min_pwm,max_pwm,0,max_thrust) #M1_PWM,M2_PWM,M3_PWM,M4_PWM
    F1 = (valmap(M2_PWM,min_pwm,max_pwm,0,max_thrust) + valmap(M3_PWM,min_pwm,max_pwm,0,max_thrust))*math.cos(math.radians(45))
    F2 = (valmap(M1_PWM,min_pwm,max_pwm,0,max_thrust) + valmap(M4_PWM,min_pwm,max_pwm,0,max_thrust))*math.cos(math.radians(45))
    net_f = (F1-F2) * arm_length #torque produced
    start_roll = ang_dis(net_f,MOI,start_roll,dt)


    DISPLAY.fill(black)
    shipImg, newRect = rot_center(img,oldRect,start_roll)
    DISPLAY.blit(shipImg, newRect)

    DISPLAY.blit(myfont.render('Angle  :  ' + str(int(start_roll)), False, (0, 255, 0)),(0,0))
    DISPLAY.blit(myfont.render('Motor 1 - PWM  :  ' + str(int(M1_PWM)), False, (255, 0, 0)),(0,20))
    DISPLAY.blit(myfont.render('Motor 2 - PWM  :  ' + str(int(M2_PWM)), False, (255, 0, 0)),(0,40))
    DISPLAY.blit(myfont.render('Motor 3 - PWM  :  ' + str(int(M3_PWM)), False, (255, 0, 0)),(0,60))
    DISPLAY.blit(myfont.render('Motor 4 - PWM  :  ' + str(int(M4_PWM)), False, (255, 0, 0)),(0,80))
    pygame.display.update()
    ctr +=1
    dt2.append(ctr*dt)
    roll_angles.append(int(start_roll))
    for event in pygame.event.get():
        if event.type==QUIT:
            print dt2
            print roll_angles
            pygame.quit()
            sys.exit()
