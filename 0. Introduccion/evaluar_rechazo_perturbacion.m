clear
close all
clc

% Parámetros
wn = 0.1;
xi = 0.01;

% Abrir modelo
open_system('planta_perturbacion.slx');

%% Caso 1: 
we = 0.1;
wn = 0.1;
sim('planta_perturbacion.slx');
t1 = yout.Time;
y1 = yout.Data;

%% Caso 2: 
we = 0.3;
wn = 0.1;
sim('planta_perturbacion.slx');
t2 = yout.Time;
y2 = yout.Data;

%% Caso 3: 
we = 0.3;
wn = 0.3;
sim('planta_perturbacion.slx');
t3 = yout.Time;
y3 = yout.Data;

%% Plot comparativo
figure
subplot(3,1,1)
plot(t1,y1,'LineWidth',1.2)
ylim([-1 1])
title('Respuesta ante perturbación coloreada')
legend('\omega_e = 0.1 , \omega_n = 0.1')
grid on
ylabel('Salida')

subplot(3,1,2)
plot(t2,y2,'LineWidth',1.2)
ylim([-1 1])
grid on
ylabel('Salida')
legend('\omega_e = 0.3 , \omega_n = 0.1')

subplot(3,1,3)
plot(t3,y3,'LineWidth',1.2)
ylim([-1 1])
grid on
xlabel('Tiempo [s]')
ylabel('Salida')
legend('\omega_e = 0.3 , \omega_n = 0.3')
