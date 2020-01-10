#!/usr/bin/env python

import pygame
import sys
import time
import random
import csv
import os

from pygame.locals import *

FPS = 10
dt = 0.1
max_num_traces = 100

pygame.init()
clock=pygame.time.Clock()

SCREEN_WIDTH, SCREEN_HEIGHT = 1900, 1000
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
surface = pygame.Surface(screen.get_size())
surface = surface.convert()
surface.fill((255,255,255))
clock = pygame.time.Clock()

vehicle_length = 4.5
num_lanes = 3
fix16_m = 65536

screen.blit(surface, (0,0))


def draw_vehicle(surf, color, pos, w, h):
    r = pygame.Rect((pos[0], pos[1]), (w, h))
    pygame.draw.rect(surf, color, r)

traces = []
if __name__ == '__main__':
  directory = sys.argv[1]
  filenames = filter(lambda s: s.endswith(".csv"), os.listdir(directory))
  max_num_traces = min(max_num_traces, len(filenames))
  filenames = random.sample(filenames, max_num_traces)

  for filename in filenames:
    f = open(directory + "/" + filename)
    reader = csv.reader(f, delimiter=",")
    traces.append([row for row in reader])
    f.close()

  max_p = 0
  for trace in traces:
    for row in trace:
      if int(row[1]) != 0:
        p = float(row[4])
        max_p = max(p + vehicle_length, max_p)

  veh_pixels_x = min(5.0, max(1.0, vehicle_length * fix16_m * float(SCREEN_WIDTH) / max_p))
  lane_height = max(1.0, veh_pixels_x / 2)

  veh_pixels_x = int(veh_pixels_x) + 1
  lane_height = int(lane_height) + 1

  font = pygame.font.Font(None, 24)
  t = -1
  
  while True:
    event = pygame.event.wait()
    if event.type == KEYDOWN:
      break

  direction = 1
  while True:
    row_indexes = range(len(traces[0]))
    if direction == -1:
      row_indexes = reversed(row_indexes)
    for row_index in row_indexes:
      for trace_index in range(len(traces)):
         row = traces[trace_index][row_index]
         if trace_index == 0 and row[2] != t:
           screen.blit(surface, (0,0))
   
           pygame.display.flip()
           pygame.display.update()
           clock.tick(FPS)
           #pygame.image.save(surface, "images/" + str(int(t) + 1).rjust(8, "0") + ".png")
           surface.fill((0, 0, 0))
           text = font.render(str(float(int(t) + 1) * dt) + " s", 30, (200, 200, 200))
           textpos = text.get_rect()
           textpos.centerx = SCREEN_WIDTH - 50
           surface.blit(text, textpos)
           screen.blit(surface, (0,0))
           t = row[2]
           for i in range(len(traces) + 1):
             pygame.draw.line(surface, (50, 50, 50), (0, i * num_lanes * lane_height), (SCREEN_WIDTH - 1, i * num_lanes * lane_height))
  
         lane = int(row[0])
         p = float(row[4])
         if trace_index % 2 == 0:
           color = (255, 0, 0)
         else:
           color = (150, 0, 0)

         if int(row[1]) == 0:
           color = (200, 200, 200)

         draw_vehicle(surface, color, (p * (SCREEN_WIDTH / max_p), trace_index * lane_height * num_lanes + lane * lane_height), veh_pixels_x, lane_height)

    if direction == 1:
      direction = -1
    else:
      direction = 1
