clc; clear all; close all;
global l V eta;

eta = 1.13e3;
eta = 578.7;
% eta = 54.3486;
% eta = 1.572e3;

tic
tempL = linspace(1e-2, 0.255, 100); % A4 half
tempL = linspace(1e-2, 0.224, 100); % A4 half
tempL = linspace(1e-2, 0.192, 100); % US half
% tempL = linspace(1e-2, 0.254, 100); % US half
% tempL = linspace(1e-2, 0.246, 100); % US half
% tempL = linspace(1e-2, 0.250, 100); % Kirigami
% tempL = linspace(1e-2, 0.195, 100); % cardboard
% tempL = linspace(1e-2, 0.266, 100); % cardboard
% tempL = linspace(1e-2, 0.275, 100); % cardBoard

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
   drawnow();
end

Vbegin  = sol1.y(3, 1);


%% folding path II
Vtarget = 80 * Vbegin;
Vtarget = -0.481; %% A4 paper half
Vtarget = -0.581; %% A4 paper half

% Vtarget = -2.081; %% US paper half
Vtarget = -2.581; %% US paper half
% Vtarget = -42.1509; %% cardBoard
% Vtarget = -0.581; %% Kirigami Paper

VForce = linspace(Vbegin, Vtarget, 1000);
sol20 = sol1;
for i = 1:length(VForce)
    V = VForce(i);
    l = sol1.y(5,1);
    sol2 = bvp5c(@bvpfun, @bcfun2, sol20);
    if norm(sol2.y(5,end) - sol20.y(5,end)) > 1e-2
            sol2 = sol20;
    end
    % check y value
    if find(sol2.y(6,:) < 0)
        if norm(sol2.y(5,end) - sol20.y(5,end)) > 1e-2
            sol2 = sol20;
        else
            sol2 = sol20;
            break;
        end
    end
    sol20 = sol2;
    FoldPath = [FoldPath; [sol2.y(5, end), sol2.y(6, end), sol2.y(1, end)]];
    drawnow();
end

L = l + abs(sol2.y(5, end) - sol2.y(5, 1));

figure(2);

plot(FoldPath(:,1), FoldPath(:,2), 'o-');

toc

% save("FoldPathA4_025.mat", "FoldPath");
% save("FoldPathUS_020.mat", "FoldPath");
save("FoldPathUS_020.mat", "FoldPath");
% save("FoldPathKirgami_030.mat", "FoldPath");




% save("FoldPathA4_030.mat", "FoldPath");
% save("FoldPathCard_027.mat", "FoldPath");


