numg = [48.83 29.3];
deng = poly ([0 -0.5858]);
G = tf(numg, deng);
T = feedback(G,1);  %T is the closed-loop TF of the PI_compensated system
T = minreal(T);
step(4*T);
axis ([0, 0.2, 0, 5]);
grid
xlabel ('Time')
ylabel ('Speed Sensor Output, c(t) in volts')
title ('PI-compensated Systems Response to a 4 volt Input Step')
pause
numr = 1;
denr = poly (0);
R = tf(numr, denr);
T2 = R*T;
E= R-T2; %E = Ramp error of PI_compensated system
step(T2,'b', R,'r', E,'g');
axis ([0, 5, 0, 5]);
grid
xlabel ('Time')
ylabel ('Output Rate, dc(t)/dt, V/s (Blue); Unit Ramp (red); Error (green)');
title ('Response of PI-compensated System to a Unit-ramp Input')