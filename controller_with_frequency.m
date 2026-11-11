numg = [1  0.6];
deng = poly ([0 -0.5858]);
G = 48.82*tf(numg, deng);
bode (G);
grid
pause
T = feedback(G,1); %T is the closed-loop TF of the PI-controlled system
T = minreal(T);
step(4*T);
axis
grid
xlabel ('Time')
ylabel ('Speed Sensor Output, c(t) in volts')
title ('PI-controlled Systems Response to a 4 volt Input Step')