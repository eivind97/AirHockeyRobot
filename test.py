import math
from datetime import datetime
from pymunk import Vec2d

vec = []

p1 = Vec2d(1400, 500)
p2 = Vec2d(500, 200)

colpos = (p1 + p2) / 2
print(colpos)

vec.append(p1)
vec.append(p2)

print(len(vec))

vel = Vec2d(-200, -120)
a_in = vel.angle
a_out = -a_in

vel2 = vel.rotated(-a_in + a_out)

print(a_in)
print(a_out)
print(vel2)