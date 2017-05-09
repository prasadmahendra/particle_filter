#!/bin/python

import math

# L1
x = 2
y = 1

# OBS1
x_mu = 0
y_mu = 5

sigma_x = 0.3
sigma_y = 0.3

C = 1.0 / (2.0 * math.pi * sigma_x * sigma_y)
x_diff = x - x_mu
y_diff = y - y_mu

e_raised = ((x_diff * x_diff) / (2.0 * sigma_x * sigma_x)) + ((y_diff * y_diff) / (2.0 * sigma_y * sigma_y))

prob = C * math.exp(-e_raised)

print(y_diff)
print(prob)
