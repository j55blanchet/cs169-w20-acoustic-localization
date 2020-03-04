#!/usr/bin/env python

# Prerequisites: have sympy installed (or use Anaconda)
import sympy

# from math import tan
from sympy.solvers.solveset import nonlinsolve
from sympy.core.symbol import symbols
from sympy import tan, pprint

x1, y1, doa_1, theta_1 = symbols('x1, y1, doa1, theta1', real=True)
x2, y2, doa_2, theta_2 = symbols('x2, y2, doa2, theta2', real=True)
x3, y3, doa_3, theta_3 = symbols('x3, y3, doa3, theta3', real=True)
xr, yr, theta_r = symbols('xr, yr, theta_r', real=True)

# eq1 = doa_1 + theta_r - theta_1 # theta 1 = theta_r + doa_1
# eq2 = doa_2 + theta_r - theta_2 # theta 2 = theta_r + doa_2
# eq3 = doa_3 + theta_r - theta_3 # theta 3 = theta_r + doa_3
# eq4 = (y1 - yr) / (x1 - xr) - tan(theta_1)
# eq5 = (y2 - yr) / (x2 - xr) - tan(theta_2)
# eq6 = (y3 - yr) / (x3 - xr) - tan(theta_3)

eq1 = (y1 - yr) / (x1 - xr) - tan(doa_1 + theta_r)
eq2 = (y2 - yr) / (x2 - xr) - tan(doa_2 + theta_r)
eq3 = (y3 - yr) / (x3 - xr) - tan(doa_3 + theta_r)

result = nonlinsolve([eq1, eq2, eq3],
                     [xr, yr, theta_r])

pprint(result, use_unicode=True)




