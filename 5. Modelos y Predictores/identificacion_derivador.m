%% Sistema a identificar
close all; clear all; clc;

wn = 0.5;
Ts = 1;
s = tf('s');
G = s/(s^2+0.5*wn*s+wn^2);
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
e = 0.1*randn(N,1);
y = idsim(th,[u e]);
z = iddata(y,u,Ts);

%% Vamos primero con arx
% Notamos que hay problemas en bajas frecuencias. Qué pasa si anulamos la
% perturbación e??

m_arx = arx(z,[2 2 1])

figure()
bode(th,m_arx)
legend('Real','Identificada')

figure()
pzmap(th,m_arx)
legend('Real','Identificada')

figure()
resid(z, m_arx)

%% Sabemos que la planta tiene un cero en DC => no lo identifiquemos!
% Vamos a identificar un modelo donde el input de DeltaU
du = diff(u);
z_dif = iddata(y(2:end),du,Ts);

m_aux = arx(z_dif,[2 1 1]);
m_arx_2 = poly2th(m_aux.A,conv(m_aux.B,[1 -1]),1,[],[],[],Ts);

figure()
bode(th,m_arx_2)
legend('Real','Identificada')

figure()
pzmap(th,m_arx_2)
legend('Real','Identificada')

figure()
resid(z_dif, m_aux)

%% Otra posibilidad era poner un integrador sobre y
y_int = cumsum(y);
z_int = iddata(y_int,u,Ts);

m_aux = arx(z_int,[2 1 1]);
m_arx_3 = poly2th(m_aux.A,conv(m_aux.B,[1 -1]),1,[],[],[],Ts);

figure()
bode(th,m_arx_3)
legend('Real','Identificada')

figure()
pzmap(th,m_arx_3)
legend('Real','Identificada')

figure()
resid(z_int, m_aux)


%% Lo anterior dio cualquier cosa. 
% Es razonable porque estamos integrando la salida, por lo que no se
% sostienen las hipótesis estadísticas sobre la perturbación. 
% Probamos cambiar el estimador a IV   
m_aux = iv4(z_int,[2 1 1]);
m_arx_4 = poly2th(m_aux.A,conv(m_aux.B,[1 -1]),1,[],[],[],Ts);

figure()
bode(th,m_arx_4)
legend('Real','Identificada')

figure()
pzmap(th,m_arx_4)
legend('Real','Identificada')

figure()
resid(z_int, m_aux)

%% Mejor buscamos una estructura para el ruido
% El modelo BJ me da mas libertad. Si configuramos para usar un integrador
% sobre el ruido tenemos este modelo
%
%                          B(q)                 C(q)      
%                  y(t) = ------ u(t-nk) +  ----------- e(t)
%                          F(q)             (1-q^-1)D(q)
%
% Como veníamos de un arx221, proponemos:
% dim(B) =1 
% dim(F) = 2
% dim(C) = 0 (una constante)
% dim(D) = 2 (debería dar la estimación de F=A)
% [nb nc nd nf nk]
m_aux = bj(z_int, [1 0 2 2 1],'IntegrateNoise',true);
m_bj = poly2th(m_aux.A,conv(m_aux.B,[1 -1]),m_aux.C,m_aux.D,m_aux.F,[],Ts);

figure()
bode(th,m_bj)
legend('Real','Identificada')

figure()
pzmap(th,m_bj)
legend('Real','Identificada')

figure()
resid(z_int, m_aux)

