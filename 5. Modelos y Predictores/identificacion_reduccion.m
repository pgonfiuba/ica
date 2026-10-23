%% Objetivo:
% Explorar la identificación de sistemas mediante modelos ARX de distinto orden
% y evaluar la reducción de orden por realización balanceada.
% Se compara la estimación directa de un modelo ARX de orden reducido con la
% identificación de un modelo de orden alto seguida de su reducción.
%
% Resultados:
% Aunque la planta real es de segundo orden, la estimación directa con un
% ARX(2,2,1) presenta una aproximación pobre de la dinámica. En cambio, la
% identificación de un modelo de orden alto y su posterior reducción permiten
% recuperar una respuesta en frecuencia más cercana a la planta original.
% Esto evidencia que el orden del modelo no garantiza por sí solo una buena
% identificación: la estructura elegida, en particular la representación del
% ruido de medición, también influye significativamente en los resultados.

%% Sistema a identificar
close all; clear all; clc;

wn = 0.5;
Ts = 1;
s = tf('s');
G = 1/(s^2+0.5*wn*s+wn^2);
Gd = c2d(G,Ts,'zoh')

% Recupero los polinomios en potencias negativas de z
[B, A] = tfdata(Gd, 'v');
C = 1;
th = poly2th(A,B,C,[],[],[],Ts);

figure()
pzmap(th)

figure()
step(G,th)
legend('Continua','Discretizada')

%% Simulación
N = 10000;
u = sign(randn(N,1));
e = 0.5*randn(N,1);
y_sr = idsim(th,u);
y = y_sr+e;


Nid = floor(0.6*N);
z_id  = iddata(y(1:Nid),u(1:Nid),Ts);
z_val = iddata(y(Nid+1:end),u(Nid+1:end),Ts);

%% Probar ARX con un orden grande

NN = struc(2:10,2:10,1:3);
V = arxstruc(z_id,z_val,NN);
nn = selstruc(V,'plot');

m_arx = arx(z_id,nn)

figure()
bode(th,m_arx)
legend('real','estimado')

figure()
pzmap(th,m_arx)
legend('real','estimado')

%% Reducir modelo
[Ad, Bd, Cd, Dd] = ssdata(m_arx);
[Ab, Bb, Cb, M, T] = dbalreal(Ad,Bd,Cd);
disp("Valores singulares de Hankel: ")
M'
% Deberían poder reducirse todos los estados menos los 2 primeros
[Ar, Br, Cr, Dr] = dmodred(Ab,Bb,Cb,Dd,3:length(M));
[b,a] = ss2tf(Ar, Br, Cr, 0);
m_arx_red = idpoly(a,b,1,1,[],[],Ts)

figure()
bode(th,m_arx_red)
legend('real','reducido')

figure()
pzmap(th,m_arx_red)
legend('real','reducido')

figure()
compare(z_val,m_arx,m_arx_red)

figure()
resid(z_val,m_arx_red)

%% Comparemos con una estimación con orden 221
m_min = arx(z_id,[2 2 1])

figure()
bode(th,m_arx_red, m_min)
legend('real','reducido','mínimo')

figure()
pzmap(th,m_arx_red, m_min)
legend('real','reducido','mínimo')

figure()
compare(z_val,m_arx,m_arx_red, m_min)

% Comparación de simulación libre: foco en la dinámica
figure
compare(z_val,m_arx,m_arx_red,m_min,Inf)

figure()
resid(z_val,m_min)

