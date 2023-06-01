clc; clear all; close all;
global l  V eta;

eta = 1.13e3; 
% eta = 578.7;
% eta = 54.3486;
% eta = 1.572e3;

tic
L = 0.27;


lr = L;
ll = 0;
xL = 0;
while abs(lr - ll) > 1e-3 && abs(xL - L) > 1e-3 
    % step 1
    tempL = linspace(1e-2, (lr + ll)/2, 100);
    FoldPath = [];
    for i = 1:length(tempL)
        l = tempL(i);
        xmesh = linspace(0, l, 100);
        if i == 1
            solinit = bvpinit(xmesh, @guess);
        else
            solinit = bvpinit(sol1, [0, l]);
        end
        sol1 = bvp5c(@bvpfun, @bcfun1, solinit);
        FoldPath = [FoldPath; [sol1.y(5, end), sol1.y(6, end), sol1.y(1, end)]];
    end
    
    Vtemp  = sol1.y(3, 1);
    % step 2
    Vtarget = 50 * Vtemp;
    Vincrement = (Vtarget - Vtemp)/1000;
    Vincrement0 = Vincrement;
    sol20 = sol1;
    hold off;
    while abs(Vtemp) < abs(Vtarget)
       V = Vtemp;
       l = sol1.y(5,1);
       sol2 = bvp5c(@bvpfun, @bcfun2, sol20);
       if sol2.stats.maxerr > 1e-2 || sol2.y(6,end) < 0
            Vtemp = Vtemp - Vincrement;
            Vincrement = 0.1 * Vincrement;
            sol2 = sol20;
       end 
       
       Vtemp = Vtemp + Vincrement;
       sol20 = sol2;
       FoldPath = [FoldPath; [sol2.y(5, end), sol2.y(6, end), sol2.y(1, end)]];
       if abs(Vincrement) <1e-7 * abs(Vincrement0) || sol2.y(6,end) == 0
          break; 
       end
    end
    
    xL = l + abs(sol2.y(5, end) - sol2.y(5, 1));
    if xL > L
        lr = (ll + lr)/2;
    else
        ll = (ll + lr)/2;
    end
    plot(FoldPath(:,1), FoldPath(:,2), 'o-');
    drawnow();
end
    
