function dydx = bvpfun(x,y)
global eta
dydx = [eta*y(2)
        y(3)
        -cos(y(1)) + eta*y(2) * y(4)
        sin(y(1)) - eta*y(2) * y(3)
        cos(y(1))
        sin(y(1))];
end