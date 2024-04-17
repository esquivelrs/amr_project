Kp_pos = 10;
Ki_pos = 1.0;
Kd_pos = 0.001;
sat_pos = 20.0;

Kp_vel = 10;
Ki_vel = 2.0;
Kd_vel = 1.0;
sat_vel = 5.0;

simout = sim("Cascaded_PID_tuning");

figure;
hold on;
plot(simout.ref,"r");
plot(simout.y,"b");
legend("ref","y");
grid on;
hold off;