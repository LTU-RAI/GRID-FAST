from sympy import *

L1x, L1y, L3x, L3y, Lpx, Lpy = symbols('L1x L1y L3x L3y Lpx Lpy ')
x12, y12, x21, y21, xp, kl, ml=symbols('x12 y12 x21 y21 xp kl ml')

#L3x=x12-xp
L3y=y12-(kl*xp+ml)
Lpx=x21-xp
Lpy=y21-(kl*xp+ml)

eq=Eq((L1x*L3x+L1y*L3y)/sqrt(L1x**2+L1y**2),(Lpx*L3x+Lpy*L3y)/sqrt(Lpx**2+Lpy**2))

print(solve(eq,L3x))
#print(simplify(eq))c