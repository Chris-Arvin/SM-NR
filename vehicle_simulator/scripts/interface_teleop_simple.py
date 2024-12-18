#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pygame
import rospy
import math
from geometry_msgs.msg import Twist

class Robot:
    def __init__(self, win_obj, id):
        self.id = id
        self.win_obj = win_obj
        self.key_pressed = [0,0]    # [move,move]
        self.pub = rospy.Publisher('/cmd_vel'+str(self.id), Twist, queue_size=10)

    def draw_obj(self, location):
        (row, col) = (location[0]*WIN_OBJ_H, location[1]*WIN_OBJ_W)
        pygame.draw.lines(self.win_obj,(128,138,135),True,((col+2,row+2),(col+198,row+2),(col+198,row+198),(col+2,row+198)),1)
        position_rec = [(col+75,row+45),(col+75,row+145),(col+25,row+95),(col+125,row+95),((col+75,row+95))]
        font_key = pygame.font.SysFont('calibri',40)    ##(.ttf)
        font_id = pygame.font.SysFont('calibri',20)
        for i in range(5):
            key = font_key.render(self.keyboard[i],True,(41,36,33))
            pygame.draw.rect(self.win_obj,(220,220,220),(position_rec[i][0],position_rec[i][1],50,50))
            self.win_obj.blit(key,(position_rec[i][0]+15,position_rec[i][1]+8))

        id = font_id.render('ID: {}'.format(self.id),True,(41,36,33))
        self.win_obj.blit(id,(position_rec[0][0],position_rec[0][1]-30))
        pygame.display.update()

    
    def set_keyboard(self,keyboard):
        self.keyboard = keyboard
        self.ww = keyboard[0]
        self.xx = keyboard[1]
        self.aa = keyboard[2]
        self.dd = keyboard[3]
        self.ss = keyboard[4]
        
    def set_publisher(self,topic):
        self.pub = rospy.Publisher(topic, Twist, queue_size=10)

    def handle_key(self):
        key_msg = Twist()
        if self.key_pressed[0] == 0:
            key_msg.linear.x = 0
        if self.key_pressed[1] == 0:
            key_msg.angular.z = 0
        if self.key_pressed[0] == self.ww:
            key_msg.linear.x = 1
        if self.key_pressed[0] == self.xx:
            key_msg.linear.x = -1
        if self.key_pressed[0] == self.ss:
            key_msg.linear.x = -2
            key_msg.angular.z = -2
        if self.key_pressed[1] == self.aa:
            key_msg.angular.z = 1
        if self.key_pressed[1] == self.dd:
            key_msg.angular.z = -1
        return key_msg

    def key_down(self, key):
        # deal with move
        if key == self.ww or key == self.ss or key == self.xx:
            self.key_pressed[0] = key
        if key == self.aa or key == self.dd:
            self.key_pressed[1] = key


    def key_up(self, key):
        # deal with move        
        if key == self.key_pressed[0]:
            self.key_pressed[0] = 0
        if key == self.key_pressed[1]:
            self.key_pressed[1] = 0 


class InterfaceSystem:
    def __init__(self,n):    
        pygame.init()
        row_num = math.ceil(n/MAX_LINE_NUM)*WIN_OBJ_H +2
        col_num = min(n,MAX_LINE_NUM)*WIN_OBJ_W +2
        self.win_obj = pygame.display.set_mode((col_num,row_num))      
        pygame.display.set_caption('Interface to Manually Control the Vehicles')   
        self.win_obj.fill((255,255,255))   
        pygame.display.flip()
        
    def getWin(self):
        return self.win_obj


WIN_OBJ_H = 199
WIN_OBJ_W = 199
MAX_LINE_NUM = 6.0
keyboard_list = [['w','x','a','d','s'],['t','b','f','h','g'],['i','m','j','l','k']]
topic_list = ["/cmd_vel".format(i+1) for i in range(len(keyboard_list))]
n = len(keyboard_list)

if __name__=="__main__":
    rospy.init_node('interface_keyboard_publisher', anonymous=True)
    unit_list = []
    
    # 行人和机器人都是手动控制
    env = InterfaceSystem(n)
    i = 0
    while i<n:
        # control robot with keyboard_teleop
        location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
        obj = Robot(env.getWin(), i+1)
        obj.set_keyboard(keyboard_list[i])
        obj.draw_obj(location)
        unit_list.append(obj)
        i+=1

    # loop to read the keyboard events
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            if event.type == pygame.KEYDOWN:
                key = None
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)
                    elif event.key==pygame.K_KP5:
                        key=chr(pygame.K_5)
                if key:
                    for unit in unit_list:
                        unit.key_down(key)

            if event.type == pygame.KEYUP:
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)

                for unit in unit_list:
                    unit.key_up(key)

        for unit in unit_list:
            key_msg = unit.handle_key()
            unit.pub.publish(key_msg)
        rate.sleep()
