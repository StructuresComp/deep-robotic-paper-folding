function res = bcfun2(ya,yb) % boundary conditions
global l V
res = [ya(1) - pi
       ya(2)
       ya(3)-V
       ya(5)-l
       ya(6)
       yb(2)];
end