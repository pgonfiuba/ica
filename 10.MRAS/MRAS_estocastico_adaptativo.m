clear all; close all; clc

%% 0. Opciones del experimento

adaptativo = true;   % true : ELS + Diofantina en cada iteración (indirecto / explícito)
                     % false: controlador fijo con parámetros nominales 

lambda = 0.99;       % factor de olvido del ELS (1 = sin olvido)
P0     = 1;          % covarianza inicial P(0) = P0*I  
b0_min = 0.1;        % cota inferior para b0 estimado (evita dividir por ~0 en la ley de control)


%% 1. Parámetros de simulación
rng(1234);

cambioref = 50;
ciclos = 5;
n = cambioref*ciclos;

e = 0.02*randn(n,1);
y = zeros(n,1);
u = zeros(n,1);

k_cambio = 1250;


%% 2. Modelo de la planta (real, el controlador NO la conoce)

% A(q^-1) y(k) = q^-nk B'(q^-1) u(k) + C(q^-1) e(k)

A = [1 -1.74 0.77];
B = [0.0011 0.0009]*1000;

na = length(A);
nb = length(B);
nk = 2;

% Modelo del ruido
C = [1 -2 1.29 -0.27];
nc = length(C);

% Parámetros nominales (antes del cambio), usados por el controlador fijo
A0 = A;
B0 = B;


%% 3. Modelo de referencia

% M(q^-1) = Bm(q^-1)/Am(q^-1)

Bm = 0.4;
Am = [1 -0.6];

nBm = length(Bm);
nAm = length(Am);

Ra  = zeros(n,1);
RaC = zeros(n,1);
ym  = zeros(n,1);


%% 4. Señal de referencia

ref = ones(cambioref,1);

for i = 1:ciclos-1
    ref = [ref; (-1)^i*ones(cambioref,1)];
end


%% 5. Estimador ELS (mínimos cuadrados extendido con olvido)

% Modelo de regresión:
%   y(k) = phi(k)' * theta + e(k)
%
%   theta = [a1 ... a_{na-1}  b0 ... b_{nb-1}  c1 ... c_{nc-1}]'
%   phi   = [-y(k-1) ... -y(k-na+1)  u(k-nk) ... u(k-nk-nb+1)  eps(k-1) ... eps(k-nc+1)]'
%
% donde eps(k) son los residuos de predicción (aproximan al ruido e(k) que no se mide).

ntheta = (na-1) + nb + (nc-1);

% Estimación inicial: deliberadamente distinta de la planta real
theta = [-1.5; 0.6;      % a1 a2
          1.0; 0.5;      % b0 b1
          0;   0;   0];  % c1 c2 c3

P = P0*eye(ntheta);

eps_hat = zeros(n,1);        % residuos a posteriori
thetaH  = zeros(ntheta,n);   % historia de estimaciones
thetaR  = zeros(ntheta,n);   % valores reales (para graficar)


%% 6. Simulación en lazo cerrado

for k = max(nk+nb+1,nc):n

    % ------------------------------------------
    % Cambio en la planta
    % ------------------------------------------

    if k == k_cambio
        B = B/2;
    end


    % ------------------------------------------
    % Planta
    % ------------------------------------------

    % A*y(k) = q^-nk*B'*u(k) + C*e(k)

    y(k) = 0;

    % Términos asociados a A
    for j = 2:na
        y(k) = y(k) - A(j)*y(k-j+1);
    end

    % Términos asociados a B'
    for j = 1:nb
        y(k) = y(k) + B(j)*u(k+1-j-nk);
    end

    % Perturbación de salida
    for j = 1:nc
        y(k) = y(k) + C(j)*e(k-j+1);
    end


    % ------------------------------------------
    % Modelo de referencia
    % ------------------------------------------

    % Referencia filtrada: Ra = Bm*r
    for j = 1:nBm
        Ra(k) = Ra(k) + Bm(j)*ref(k-j+1);
    end

    % Dinámica del modelo: Am*ym = Ra
    ym(k) = Ra(k);

    for j = 2:nAm
        ym(k) = ym(k) - Am(j)*ym(k-j+1);
    end


    % ------------------------------------------
    % Identificación: ELS (usa y(k) recién medida)
    % ------------------------------------------

    phi = [-y(k-1:-1:k-na+1);
            u(k-nk:-1:k-nk-nb+1);
            eps_hat(k-1:-1:k-nc+1)];

    err_ap = y(k) - phi'*theta;                 % error a priori

    K     = P*phi/(lambda + phi'*P*phi);        % ganancia
    theta = theta + K*err_ap;                   % actualización de parámetros
    P     = (P - K*(phi'*P))/lambda;            % actualización de covarianza

    eps_hat(k) = y(k) - phi'*theta;             % residuo a posteriori

    thetaH(:,k) = theta;
    thetaR(:,k) = [A(2:end)'; B(:); C(2:end)'];


    % ------------------------------------------
    % Polinomios que usa el controlador
    % ------------------------------------------

    if adaptativo
        Ah = [1, theta(1:na-1)'];
        Bh = theta(na:na+nb-1)';
        Ch = [1, theta(na+nb:end)'];
    else
        Ah = A0;
        Bh = B0;
        Ch = C;
    end


    % ------------------------------------------
    % Diofantina en línea:  Ch*Am = F*Ah + q^-nk*G
    % ------------------------------------------

    CAm = conv(Ch,Am);
    [F,G] = diofantina(Ah,CAm,nk);

    FB  = conv(F,Bh);      % F*B'
    CBm = conv(Ch,Bm);     % C*Bm

    b0 = FB(1);
    if abs(b0) < b0_min    % protección: b0 estimado ~ 0 o con signo cambiado
        b0 = b0_min;
    end


    % ------------------------------------------
    % Referencia del controlador
    % ------------------------------------------

    % RaC = Ch*Bm*r

    for j = 1:length(CBm)
        if k-j+1 > 0
            RaC(k) = RaC(k) + CBm(j)*ref(k-j+1);
        end
    end


    % ------------------------------------------
    % Ley de control
    % ------------------------------------------

    % FB'*u(k) = CBm*r(k) - G*y(k)

    u(k) = ( RaC(k) -G*y(k:-1:k-length(G)+1) - FB(2:end)*u(k-1:-1:k-length(FB)+1) ) / b0;

end


%% 7. Resultados

t = 0:length(ym)-1;

figure;

subplot(2,1,1)

plot(t,y,'b','LineWidth',1.5);
hold on;

plot(t(nk+1:end),ym(1:length(ym)-nk),...
    'k--','LineWidth',1.2);

axis([0 n -1.5 1.5]);
grid on;

legend('y','y^M','Location','southeast');
ylabel('y');

if adaptativo
    title('MRAS estocástico adaptativo indirecto (ELS)');
else
    title('MRAS estocástico con parámetros fijos');
end

subplot(2,1,2)

stairs(u,'r');

grid on;
xlabel('Muestra');
ylabel('u');


%% Error de seguimiento filtrado
yf  = filter(Am(1:nAm),1,y);
ymf = filter(Am(1:nAm),1,ym);

error_filtrado = yf - ymf;

figure;
plot(error_filtrado);
hold on
plot([k_cambio k_cambio], ylim, ':r', 'HandleVisibility','off');
grid on;
xlabel('Muestra');
ylabel('Error filtrado');

% Varianza del error filtrado en tramos donde la referencia ya está en
% régimen (se evitan los 10 primeros instantes después de cada escalón):
%   antes del cambio     : 60:95   (ref = -1)
%   después del cambio   : 165:195 (ref = -1)  y  215:245 (ref = +1)
fprintf('var(error filtrado) antes del cambio    [60:95]   = %.5f\n', var(error_filtrado(60:95)));
fprintf('var(error filtrado) despues del cambio  [165:195] = %.5f\n', var(error_filtrado(165:195)));
fprintf('var(error filtrado) despues del cambio  [215:245] = %.5f\n', var(error_filtrado(215:245)));


%% Evolución de los parámetros estimados (línea punteada = valor real)

figure;

subplot(3,1,1)
plot(thetaH(1:na-1,:)','LineWidth',1.5); hold on;
set(gca,'ColorOrderIndex',1);
plot(thetaR(1:na-1,:)','--k');
grid on; ylabel('a_i');
title('Parámetros estimados (ELS) vs reales');

subplot(3,1,2)
plot(thetaH(na:na+nb-1,:)','LineWidth',1.5); hold on;
set(gca,'ColorOrderIndex',1);
plot(thetaR(na:na+nb-1,:)','--k');
grid on; ylabel('b_i');

subplot(3,1,3)
plot(thetaH(na+nb:end,:)','LineWidth',1.5); hold on;
set(gca,'ColorOrderIndex',1);
plot(thetaR(na+nb:end,:)','--k');
grid on; ylabel('c_i'); xlabel('Muestra');


%% Funciones locales 

function [F,G] = diofantina(A,CAm,nk)
% Resuelve  CAm = F*A + q^-nk * G
% A y CAm: vectores fila, A mónico. F tiene grado nk-1.

    F = zeros(1,nk);
    F(1) = 1;

    for i = 2:nk
        F(i) = CAm(i);
        for j = 1:i-1
            if i-j+1 <= length(A)
                F(i) = F(i) - F(j)*A(i-j+1);
            end
        end
    end

    FA = conv(F,A);

    L = max(length(CAm),length(FA));

    residuo = zeros(1,L);
    residuo(1:length(CAm)) = residuo(1:length(CAm)) + CAm;
    residuo(1:length(FA))  = residuo(1:length(FA))  - FA;

    G = residuo(nk+1:end);

end