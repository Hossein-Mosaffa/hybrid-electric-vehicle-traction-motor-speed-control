% Root-locus analysis of a third-order hybrid-electric-vehicle
% traction-motor speed-control loop.

numg = poly([-0.4 -0.6]);
deng = poly([0 -0.0163 -0.5858]);

G = tf(numg, deng);

rlocus(G);

pos = 10;   % Desired percent overshoot
z = -log(pos/100) / sqrt(pi^2 + (log(pos/100))^2);

axis([-1 0 -0.5 0.5])
sgrid(z, 0)
title(['Root Locus with ', num2str(pos), ' Percent Overshoot Line'])

% Use the same gain as frequency_responce.m so that both scripts
% describe the same closed-loop design.
K = 0.78;

% To choose the gain interactively instead, comment out the line above
% and uncomment the following two lines:
% [K, poles] = rlocfind(G);
% disp(['Selected gain: ', num2str(K)])

T = feedback(K * G, 1);   % Closed-loop transfer function
T = minreal(T);

step(T);
axis([0 20 0 1.5])
grid
