t = tout;
RTD = 180 / pi;

figure(1)
plot(t,theta * RTD);
grid on
hold on
plot(t,theta_d * RTD);
xlabel('t/s');
ylabel('\theta/°');
title('俯仰角随时间变化曲线');
legend('实际','期望');

figure(2)
plot(t,phi * RTD);
grid on
hold on
plot(t,phi_d * RTD);
xlabel('t/s');
ylabel('\phi/°');
title('偏航角随时间变化曲线');
legend('实际','期望');

figure(3)
plot(t,gamma * RTD);
grid on
hold on
plot(t,gamma_d * RTD);
xlabel('t/s');
ylabel('\gamma/°');
title('滚转角随时间变化曲线');
legend('实际','期望');

figure(4)
plot(t,M_x,t,M_y,t,M_z);
grid on
xlabel('t/s');
ylabel('力矩/N·m');
title('力矩随时间变化曲线');
legend('M_x','M_y','M_z');

figure(5)
plot(t,omega(:,1));
grid on
hold on
plot(t,omega_r(:,1));
xlabel('t/s');
ylabel('\omega_x/rad·s-1');
title('滚转角速度随时间变化曲线');
legend('实际','期望');

figure(6)
plot(t,omega(:,2));
grid on
hold on
plot(t,omega_r(:,2));
xlabel('t/s');
ylabel('\omega_y/rad·s-1');
title('偏航角速度随时间变化曲线');
legend('实际','期望');

figure(7)
plot(t,omega(:,3));
grid on
hold on
plot(t,omega_r(:,3));
xlabel('t/s');
ylabel('\omega_z/rad·s-1');
title('俯仰角速度随时间变化曲线');
legend('实际','期望');
