clear all; close all; clc

%% Control predictivo basado en polinomios
rng(1234)

% --- Parámetros de la referencia y señales ---
cambioref = 50;           % tamaño del tramo de referencia constante
ciclos = 5;               % número de ciclos de referencia
n = cambioref * ciclos;   % longitud total del experimento

e = 0.02*randn(n,1);       % ruido de medida 
y = zeros(n,1);           % salida del proceso
u = zeros(n,1);           % señal de control
uf = zeros(n,1);          % señal de control filtrada (para término R/P)

% --- Modelo del proceso (polinomios en z^-1) ---
Ap = [1 -1.74 0.77];      % polinomio A (denominador del modelo)
Bp = 2 * [1.1 0.9];       % polinomio B (numerador del modelo), con factor 2
A = Ap;
B = Bp;
na = length(Ap);
nb = length(Bp);

% --- Polinomio de perturbación / ruido C y orden nc ---
nc = 4;
C = [1.0000;
     -2.0000;
      1.2900;
     -0.2700];

nk = 2;                   % retraso muerto (delay) en muestras

% --- Referencia: vector con segmentos alternados +/-1 ---
ref = ones(cambioref,1);
for i = 1:ciclos-1
    ref = [ref; (-1)^i * ones(cambioref,1)];
end

% --- Parámetros del regulador / ponderación ---
lambda = 10;              % parámetro de regularización (peso)
% Polinomios P y R (filtros del término de control)
P = [1 -1.3981 0.4976];
R = [1 -0.9473 0.1205];
R=[1, -1, 0]
% --- Cálculo del vector F (relacionado con la expansión de 1/A truncada) ---
F = zeros(nk,1);
F(1) = 1;
for i = 2:nk
    for j = 1:i-1
        if i-j+1 <= length(A)
            F(i) = F(i) - F(j) * A(i-j+1);
        end
    end
end

% --- Cálculo del vector G (coeficientes que relacionan A y F) ---
G = zeros(na,1);
for i = 1:na
    for j = 0:nk-1
        if i + nk - j <= length(A)
            G(i) = G(i) - F(j+1) * A(i + nk - j);
        end
    end
end

% --- Convolución de F y B para obtener FB (usado en parámetros) ---
FB = conv(F, B)';

% --- Cálculo del vector de parámetros theta que determina la acción de control ---
npar = na + nb + nk - 3 + length(P);   % tamaño del vector de parámetros
theta = ones(npar,1) * B(1) / (B(1)*B(1) + lambda);

% relleno de componentes de theta según la formulación matricial
theta(2:na) = theta(1) * G(1:na-1);
theta(na+1 : na+nb+nk-2) = (FB(2:nb+nk-1) + lambda * R(2:nb+nk-1)' / B(1)) * theta(1);
theta(na+nb+nk-1 : na+nb+nk+length(P)-3) = -lambda * P(2:length(P)) / B(1) * theta(1);

% --- Bucle principal de simulación / control predictivo ---
% Se empieza desde un índice que evita acceder a elementos negativos
for i = nk + nb + 5 : n
    % --- Dinámica del proceso: y(i) calculada a partir de A, B, y perturbación C*e ---
    y(i) = 0;
    for j = 2:na
        y(i) = y(i) - A(j) * y(i - j + 1);
    end
    for j = 1:nb
        y(i) = y(i) + B(j) * u(i + 1 - j - nk);
    end
    for j = 1:nc
        y(i) = y(i) + C(j) * e(i - j);
    end

    % --- Regulador: cálculo de uf (control filtrado por R/P) ---
    uf(i-1) = 0;
    for j = 1:length(R)
        uf(i-1) = uf(i-1) + u(i-j) * R(j);
    end
    for j = 2:length(P)
        uf(i-1) = uf(i-1) - uf(i-j) * P(j);
    end

    % --- Construcción del vector de valores ValReg usado para calcular u(i) ---
    phi_reg = zeros(npar,1);
    phi_reg(1) = ref(i);                                       % referencia futura inmediata
    phi_reg(2:na) = -flipud(y(i - na + 2 : i));                % salidas pasadas (signo -)
    phi_reg(na+1 : na+nb+nk-2) = -flipud(u(i - nb - nk + 2 : i - 1));  % controles pasados
    phi_reg(na+nb+nk-1 : na+nb+nk+length(P)-3) = -flipud(uf(i - length(P) + 1 : i - 1)); % uf pasado

    Aest(i, :) = theta';
    
    % --- Acción de control: producto interno entre phi_reg y theta ---
    u(i) = phi_reg' * theta;
end


%% Resultados
t = 0:length(y)-1;

figure;

subplot(2,1,1)
plot(t,y,'b','LineWidth',1.5);
hold on;
plot(t(nk+1:end),ref(1:length(ref)-nk),'k--','LineWidth',1.2);
axis([0 n -1.5 1.5]);
grid on;
legend('y','r','Location','southeast');
ylabel('y');

subplot(2,1,2)
stairs(u,'r');
axis([0 n -2 2]);
grid on;
xlabel('Muestra');
ylabel('u');

figure;
plot(Aest,'LineWidth',1.5); grid on;
title('Evolución de parámetros estimados');
xlabel('Muestra');
ylabel('Parámetros');