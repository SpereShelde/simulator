from cvxpy import *


x = Variable()
u = Variable()
j = x+u
t = fix(j, [x])