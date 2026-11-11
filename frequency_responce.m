% Frequency response and step response of a PI-compensated
% hybrid-electric-vehicle traction-motor speed-control loop.

numg = [0.78 0.78 0.1872];
deng = poly([0 -0.0163 -0.5858]);

K = 0.78;   % Gain selected from the root-locus design
G = K * tf(numg, deng);

bode(G);
grid
pause

T = feedback(G, 1);   % Closed-loop transfer function
T = minreal(T);

step(4 * T);
grid
xlabel('Time')
ylabel('Speed Sensor Output, c(t) in volts')
title('PI-Controlled Traction-Motor Speed Response to a 4 V Step')
