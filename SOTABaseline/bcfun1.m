function res = bcfun1(ya,yb) % boundary conditions
global l
res = [ya(1) - pi
       ya(2)
       ya(4)
       ya(5)-l
       ya(6)
       yb(2)];
end