%% Ejemplo de estimación por LS de un sistema muy simple
Ts = 1;
G = ss(0.5,1,1,0,Ts);

% Proponemos una delta
% Excitación: delta
N = 10;
u = zeros(N,1);
u(1) = 1;
t = (0:N-1)*Ts;
y = lsim(G,u,t,0);

stem(y)

% Modelo ARX: y(k) = a*y(k-1) + b*u(k-1)
Y   = y(2:N);
Phi = [y(1:N-1) u(1:N-1)];

% Mínimos cuadrados
theta = Phi\Y

% Número de condición
cond(Phi'*Phi)

%% Volvemos a repetir con una excitación escalón
u = ones(N,1);
y = lsim(G,u,t,0);

Y   = y(2:N);
Phi = [y(1:N-1) u(1:N-1)];

% Mínimos cuadrados
theta = Phi\Y

% Número de condición
cond(Phi'*Phi)

%% Vamos con un sistema de segundo orden
Ts = 1;

% Sistema de segundo orden
A = [1.2 -0.32;
     1    0];
B = [1; 0];
C = [1 0];
D = 0;

G = ss(A,B,C,D,Ts);

% Excitación: delta
N = 10;
u = zeros(N,1);
u(1) = 1;

t = (0:N-1)*Ts;
y = lsim(G,u,t,[0,0]);

stem(t,y)

% Modelo ARX:
% y(k) = a1*y(k-1) + a2*y(k-2) + b1*u(k-1)

Y   = y(3:N);
Phi = [y(2:N-1) y(1:N-2) u(2:N-1)];

% Rango de la matriz de regresores
Phi
rank(Phi)

% Número de condición
cond(Phi'*Phi)

% Mínimos cuadrados
theta = Phi\Y

%% Repetimos para el segundo orden pero con escalón
N = 10;
u = ones(N,1);
y = lsim(G,u,t,[0,0]);

stem(t,y)

Y = y(3:N);
Phi = [y(2:N-1) y(1:N-2) u(2:N-1)];

% Rango de la matriz de regresores
rank(Phi)

% Número de condición
cond(Phi'*Phi)

% Mínimos cuadrados
theta = Phi\Y
