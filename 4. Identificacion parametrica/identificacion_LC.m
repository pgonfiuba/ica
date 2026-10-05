%% Identificación en Lazo Cerrado de un sistema con un integrador puro

clc; clear all; close all;

% Vamos con una planta con un polo en s=-1 y un integrador puro
s = tf('s');
G = 1/((s+1)*s);

% Muestreamos con Ts=1
Ts = 1;
Gd = c2d(G,Ts,'zoh');

% Recupero los polinomios en potencias negativas de z
[B, A] = tfdata(Gd, 'v');
C = 1;

% Armo el modelo ARX para comparar (ground truth)
th = poly2th(A,B,C)

% Simulación en LC con un control Kp=1
Kp = 1;
N = 2000;

% Ruido blanco de medición
rb = 0.1*randn(N,1);

% Referencia
r = sign(randn(N,1));

ym = zeros(N,1);
u  = zeros(N,1);
y  = zeros(N,1);

% Simulación discreta de la planta
for k = 3:N
    % La salida medida se ve afectada por el ruido de medición
    ym(k-1) = y(k-1) + rb(k-1);
    
    % Calculo la acción de control
    u(k-1)  = Kp*(r(k-1) - ym(k-1));
    
    % Calculo la evolución del sistema
    y(k) = -A(2)*y(k-1) - A(3)*y(k-2) + B(2)*u(k-1) + B(3)*u(k-2);
end
% Ultima muestra
ym(N) = y(N) + rb(N);


%% Identificación usando arx 221
z = iddata(ym,u,1);
arx221 = arx(z,[2 2 1])

figure
pzmap(th,arx221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

figure
bode(th,arx221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

%% Identificación usando iv 221
z = iddata(ym,u,1);
iv221 = iv4(z,[2 2 1])

figure
pzmap(th,iv221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

figure
bode(th,iv221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

%% Identificación usando armax 2221

armax221 = armax(z,[2 2 2 1])

figure
pzmap(th,armax221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

figure
bode(th,armax221)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

%% Cómo resolvemos el polo inestable??
% La identificación se efuerza por descubrir el polo en z=1
% En el caso insesgado, IV4 y ARMAX tienen una precisión que disminuye
% conforme aumenta N.
% Con una tira de datos finita, es esperable que el polo en z=1 pueda
% quedar fuera del círculo unidad
% En este caso es mejor imponer la estructura de las características
% conocidas. Si tomamos la salida derivada, estaremos identificando hasta
% antes del integrador

ym_f = filter([1 -1],1,ym);
z = iddata(ym_f,u,1);
iv_f = iv4(z,[1 2 1])

Giv = idpoly(conv(iv_f.A,[1 -1]), iv_f.B);
Giv.Ts = 1;

figure
pzmap(th,Giv)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')

figure
bode(th,Giv)
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
legend('Real','arx221')
