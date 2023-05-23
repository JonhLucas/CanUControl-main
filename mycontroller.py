#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 22:38:02 2023

@author: jonhlucas
"""

from math import *
from server_base import Server
import numpy as np
import csv
import time

dt = 0.06;
points = []

class Control:

	def __init__(self, verbose = False):

		self.verbose = verbose
		self.time = 0.0
		self.lastControlSignalSent = 0
		self.referenceError = 0
		self.lastResetTime = time.time()
		self.moving = True
		self.right = False
		self.left = False

	def step(self, received):

		print(received)
		
		controlSignal = sin(self.time)
		
		if received:
			try:
					
				land, level = received[1], received[2]
				character_x, character_y, target_x, target_y = received[3], received[4], received[5], received[6]
				
				r = np.array([[target_x], [target_y]])
				x = np.array([[character_x], [character_y]])
				
				if land == 1:


					if(level == 1):
						K = np.array([8.5111, 0.6676])
						self.referenceError += (r-x) * 0.03
						Ke = 1
						T = 0.7

					if(level == 2):
						K = np.array([8.3607, 4.0335])
						self.referenceError += (r-x) * 0.03
						Ke = 1
						T = 0.5
						
					if(level == 3):
						K = np.array([-9.0385, -2.7900])
						self.referenceError = (r-x)
						Ke = 1
						T = 0.7
						
					if(level == 4):
						K = np.array([12.4498, -0.7476])
						self.referenceError = (r-x)
						Ke = 1	
						T = 0.7
				elif land == 2:
					if level == 1.0:

						bins = np.array([-np.pi * 3/8, -np.pi/8 - 0.03, np.pi/8 + 0.03, np.pi * 3/8])
						inds_target = np.digitize(target_x, bins)
						inds_character = np.digitize(character_x, bins)
						print(inds_target)
						if inds_character < inds_target and not self.left:
							print('direita', target_x, character_x, target_x - character_x, target_y)
							if character_y < 0.55 and self.moving:
								self.right = True
								return -1
							#elif np.abs(target_x - character_x) > 0.05:
							else:
								self.moving = False
								return 1
						elif inds_character > inds_target and not self.right:
							print('Esquerda', target_x, character_x, target_x - character_x, character_y)
							if character_y > -0.55 and self.moving:
								self.left = True
								return 1
							#elif np.abs(target_x - character_x) > 0.05:
							else:
								self.moving = False
								return -1
						elif np.abs(character_y) > 0.5 and not self.moving:
							print('ajustando', character_y)
							return 0
						else:
							self.moving = True
							self.right = False
							self.left = False
							return 0
							
					elif level == 2.0:
						self.moving = True
						self.right = False
						self.left = False
						
						error = r - x
						K = np.array([0.7599217,   0.6542919])
						v = (K @ error) * 3
						u = (1/2) * (v - np.arctan(3 * character_x))
						controlSignal = np.clip(u, -1, 1)[0]
						print(u, error)
						return controlSignal
					elif level == 3.0:
						if r[0] > 0.4:
							r[0] = 0
							r[1] = 0
						error = r - x
						K = np.array([1.2372688,   0.1769447])
						v = (K @ error) * 4
						print(K, error, v)
						controlSignal = np.clip(v, -1, 0.7)[0]
						return controlSignal
					elif level == 4.0:
						
						if(r[0] < 0): #Esperar entrar numa área acessível
							return 1
						elif(r[0] < 0.4 and r[1] > x[1]):#
							return 1

						p = 20
						K = np.array([0.1609899,   0.6034566])
						c = x.copy()
						t = r.copy()
						#transformação de coordenadas
						c[0] -= 0.7
						t[0] -= 0.7
						

						#Controlador
						v = K @ c #realimentação
						error = t - c #Error
						u = p * error[0] - v
						controlSignal = np.clip(u, -1, 1)[0]
						print(c.T, t.T, u) 
						return controlSignal
					
				elif land == 3:
				

					r = np.array([[target_x], [target_y]])
					x = np.array([[character_x], [character_y]])

					if(level == 1):
						K = np.array([10.8965, 5.3830])
						E = np.array([[1], [0]])
						if(x[0] < 0.0):
							E = np.array([[-1], [0]])				
						self.referenceError = (r-x) * 0.03
						Ke = 10
						T = 0.7
						
					if(level == 2):
						K = np.array([-10.1258, 10.4941])
						if(x[0] < 0.0):
							K = np.array([10.1258, -10.4941])		
						self.referenceError = (r-x) * 0.03
						Ke = 10
						T = 0.7
					
					if(level == 3):
						K = np.array([-8.1980, 11.5235])
						E = np.array([[0], [1]])
						if(x[0] < 0.0): 
							K = np.array([8.1980, -11.5235])
							E = np.array([[0], [-1]])		
						self.referenceError = (r-x) * 0.03
						Ke = 1
						T = 0.15
						
					if(level == 4):
						K = np.array([13.0014, -14.4652])
						if(-0.1582*x[0] + 1.8467*x[1] < 0.0): 
							K = np.array([-13.0014, 14.4652])	
						self.referenceError = (r-x) * 0.03
						Ke = 10
						T = 1.5
				
				
				else:
					print("\nFora de alcance\n")
					controlSignal = self.lastControlSignalSent
					
				u = Ke * K @ (self.referenceError)
				s = u

				controlSignal = np.clip(s, -1, 1)[0]
				self.lastControlSignalSent = controlSignal

				currentTime = time.time()
				if currentTime - self.lastResetTime > T:
					self.referenceError = 0
					self.lastResetTime = currentTime
			except Exception as ex:
				print(ex)
				
		self.time += dt
		return controlSignal


if __name__=='__main__':

	control = Control()

	server = Server(control)

	server.run()








