%% Máquina de papel muestreada con T=1
% x_dot(t) = -x(t) + u(t-2.5)

close all
clear all

Phi = [0.37 0.24 0.39 0;0 0 1 0; 0 0 0 1; 0 0 0 0]
Gamma = [0 0 0 1]'
C = [1 0 0 0 ]
D=0
T=1

Tfin = 20

Hss = ss(Phi, Gamma, C, D, T);
H = minreal(tf(Hss))
[p,z] = pzmap(H)
[y_ol,t_ol]=step(H,Tfin);

%%  Vamos con un triste control proporcional
% Lamentablemente vamos a tener que bajar las pretensiones de performance
% Tenemos:
% - 3 polos en cero (el retardo)
% - 1 polo en 0.37 (por la dinámica de la planta)
% - 1 cero en -0.61538 por el muestreo
% No se puede cancelar el cero de muestreo, a menos que quieras oscilaciones ocultas!! -> igual no
% Al dar un poco de ganancia Kp un par de polos que están en cero van hacia el cero con una amortiguación muuuuy baja
% Pero como son rápidos, con poca ganancia no se nota tanto su efecto
rlocus(H)
Control = 0.1
Hcl_rl = Control*H / (1+Control*H); 
Kr = 1/dcgain(Hcl_rl);
Hcl_rl = minreal(Hcl_rl * Kr)
Ucl_rl = minreal(Hcl_rl/H);
[p,z] = pzmap(Hcl_rl)
[y,t] = step(Hcl_rl,Tfin);
[u,t] = step(Ucl_rl,Tfin);

figure
subplot(2,1,1)
stairs(t, y, 'linewidth', 2)
hold on
plot(t, y, 'o')
stairs(t_ol, y_ol, 'linewidth', 2)
plot(t_ol, y_ol, 'o')
grid on
ylabel('y')
title('Respuesta al escalón: lazo abierto vs. control proporcional')
legend('Lazo cerrado', '', ...
       'Lazo abierto', '', ...
       'location', 'southeast')
subplot(2,1,2)
stairs(t, u, 'linewidth', 2)
hold on
plot(t, u, 'o')
xlabel('Tiempo [s]')
ylabel('u')       
grid on

%%  Nos tiramos de cabeza con un deadbeat que cancele las dinámicas del sistema muestreado
[polo,zero] = pzmap(H);
Kp = 0.1
Control = zpk(polo(1),zero,Kp,T)
Hcl = minreal(Control*H/(1+Control*H));
Kr = 1/ dcgain(Hcl);
Hcl = Hcl * Kr
U = Hcl/H

% Simulamos las respuestas
[y,t]=step(Hcl,Tfin);
[u,t]=step(U,Tfin);
       
figure
subplot(2,1,1)
stairs(t, y, 'linewidth', 2)
hold on
plot(t, y, 'o')
stairs(t_ol, y_ol, 'linewidth', 2)
plot(t_ol, y_ol, 'o')
grid on
ylabel('y')
title('Respuesta al escalón: la vs. lc (deadbeat c/ cancel)')
legend('Lazo cerrado', '', ...
       'Lazo abierto', '', ...
       'location', 'southeast')
subplot(2,1,2)
stairs(t, u, 'linewidth', 2)
hold on
plot(t, u, 'o')
xlabel('Tiempo [s]')
ylabel('u')       
grid on


%% Diseño por colocación de polos 
% Primero verificamos controlabilidad!
Co = ctrb(Phi,Gamma);
rank(Co)

% Colocamos polos
% Solo un poco más rápido que la planta
%p = [0.01 0.012 0.015 0.3]

% Mucho más rápido
%p = [0.01 0.01 0.015 0.1]

% Deadbeat: velocidad extrema y maximo esfuerzo del control!
p = [0.0 0.0 0.0 0.0]

K = acker(Phi, Gamma, p);

% Nueva matriz de estado
Acl = Phi - Gamma*K;
% Corroboro ubicación de polos
eig(Acl)

% Calculo la ganancia de continua (z=1) para tener seguimiento 
Kr_0 = 1 / (C * inv(eye(4) - Acl) * Gamma);

Hcl_ss = ss(Acl, Gamma*Kr_0, C, D, T);
% Dinámica del sistema de lc
Hcl = minreal(tf(Hcl_ss))

% Pero veamos también el esfuerzo del control ...
% Transferencia R -> U: notar que la matriz D no es cero ... tipico del control!
Uss = ss(Acl, Gamma*Kr_0, -K, Kr_0, T);
% Convertir a TF (opcional)
U = tf(Uss);

% Simulamos las respuestas
[y,t]=step(Hcl,Tfin);
[u,t]=step(U,Tfin);
       
figure
subplot(2,1,1)
stairs(t, y, 'linewidth', 2)
hold on
plot(t, y, 'o')
stairs(t_ol, y_ol, 'linewidth', 2)
plot(t_ol, y_ol, 'o')
grid on
ylabel('y')
title('Respuesta al escalón: la vs. lc (colocación)')
legend('Lazo cerrado', '', ...
       'Lazo abierto', '', ...
       'location', 'southeast')
subplot(2,1,2)
stairs(t, u, 'linewidth', 2)
hold on
plot(t, u, 'o')
xlabel('Tiempo [s]')
ylabel('u')       
grid on


% Veamos el controlador que quedó armado, pensado como transferencias
% u = - Kx *x + Kr0*r 
% u[k] = -k1 y[k] -k2 u[k-3] -k3 u[k-2] -k1 u[k-1] + Kr0 r[k]
% u[k] +k2 u[k-3] +k3 u[k-2] +k1 u[k-1] = -k1 y[k] + Kr0 r[k]
% Q u[k] = k1 y[k] + Kr0 r[k]
% Q/(-k1) u[k] = y[k] + Kr0/k1 r[k]
% u[k] = (-k1/Q) * (y[k] + Kr0/k1 r[k])
%        CONTROL           PREFILTRO
z = tf('z', T);
Q = 1 + K(4)*z^(-1) + K(3)*z^(-2) + K(2)*z^(-3);
Control = minreal(K(1)/Q)
Kr = Kr_0/K(1)
