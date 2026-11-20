
clear all; close all; clc

%% Control Predictivo Ponderado por Polinomios (versión adaptativa)
rng(1234)

% ----- Configuración del experimento -----
cambioref = 30;                 % duración de cada tramo de referencia
ciclos = 5;                     % número de ciclos
n = cambioref * ciclos;         % longitud total del experimento

% Ruido de medida y señales iniciales
e = 0.02 * randn(n,1);          % ruido (vector)
y = zeros(n,1);                 % salida del proceso
u = zeros(n,1);                 % actuación
uf = zeros(n,1);                % actuación filtrada (R/P)

% ----- Modelo nominal (polinomios en z^-1) -----
Ap = [1 -1.74 0.77];            % polinomio A
Bp = 2 * [1.1 0.9];             % polinomio B (factor 2)
A = Ap;
B = Bp;
na = length(Ap);
nb = length(Bp);

% ----- Polinomio de perturbación/ruido C -----
nc = 4;
C = [1.0000;
    -2.0000;
     1.2900;
    -0.2700];

nk = 2;                         % retraso muerto en muestras

% ----- Referencia: secuencia alternada +/-1 -----
ref = ones(cambioref,1);
for ii = 1:(ciclos-1)
    ref = [ref; (-1)^ii * ones(cambioref,1)];
end

% ----- Polinomios de ponderación P y R -----
P = [1 -1.3981 0.4976];         % polinomio P (filtro en denominador de uf)
R = [1 -0.9473 0.1205];         % polinomio R (filtro en numerador de uf)
lambda = 10;                    % parámetro de regularización/ponderación

% ----- Inicialización identificación adaptativa -----
% número de parámetros (regresores) usados en el identificador / regulador
np = na + nb + nk - 3 + length(P);   % coincide con la formulación matricial
Aest = ones(n, np);         % estimaciones de parámetros a lo largo del tiempo
lam = 0.99;                 % factor de olvido (lambda)
Pmat = 10000 * eye(np);     % matriz de covarianza inicial (p)
th = eps * ones(np,1);      % vector de parámetros pequeño inicial (no usado directamente)

% ----- Cálculo del polinomio F (expansión truncada de 1/A) -----
F = zeros(nk,1);
F(1) = 1;
for i = 2:nk
    for j = 1:(i-1)
        if i-j+1 <= length(A)
            F(i) = F(i) - F(j) * A(i-j+1);
        end
    end
end

% ----- Cálculo del vector G (relación entre A y F) -----
G = zeros(na,1);
for i = 1:na
    for j = 0:(nk-1)
        if i + nk - j <= length(A)
            G(i) = G(i) - F(j+1) * A(i + nk - j);
        end
    end
end

% ----- Convolución F * B (usado en formulación del regulador) -----
FB = conv(F, B)';

% ----- Vector theta (inicialización, no es usado directamente en la versión adaptativa
% pero se calcula por completitud) -----
theta = ones(np,1) * B(1) / (B(1)*B(1) + lambda);
theta(2:na) = theta(1) * G(1:na-1);
theta(na+1 : na+nb+nk-2) = (FB(2:nb+nk-1) + lambda * R(2:nb+nk-1)' / B(1)) * theta(1);
theta(na+nb+nk-1 : na+nb+nk+length(P)-3) = -lambda * P(2:length(P)) / B(1) * theta(1);

% ----- Bucle principal: simulación + regulación + identificación -----
% Se empieza en un índice que evita acceder a índices <= 0
startIdx = nk + nb + 5;
for i = startIdx : n

    % ----- Dinámica de la planta (simulación) -----
    y(i) = 0;
    % parte autoregresiva (A)
    for j = 2:na
        y(i) = y(i) - A(j) * y(i - j + 1);
    end
    % parte dependiente de entrada (B con retraso nk)
    for j = 1:nb
        y(i) = y(i) + B(j) * u(i + 1 - j - nk);
    end
    % término de perturbación/ruido (C * e)
    for j = 1:nc
        % proteger índices negativos: exigir i-j >= 1
        if (i - j) >= 1
            y(i) = y(i) + C(j) * e(i - j);
        end
    end

    % ----- Regulador: cálculo de uf = (R/P) * u (filtro aplicado a u) -----
    uf(i-1) = 0;
    for j = 1:length(R)
        if (i - j) >= 1
            uf(i-1) = uf(i-1) + u(i - j) * R(j);
        end
    end
    for j = 2:length(P)
        if (i - j) >= 1
            uf(i-1) = uf(i-1) - uf(i - j) * P(j);
        end
    end

    % ----- Construcción del vector de regresores phi (entrada del regulador) -----
    phi = zeros(np,1);
    phi(1) = ref(i);                                  % referencia futura inmediata
    % salidas pasadas con signo negativo
    phi(2:na) = -flipud( y(i - na + 2 : i) );
    % controles pasados (u)
    phi(na+1 : na+nb+nk-2) = -flipud( u(i - nb - nk + 2 : i - 1) );
    % uf pasado (filtrado)
    phi(na+nb+nk-1 : na+nb+nk+length(P)-3) = -flipud( uf(i - length(P) + 1 : i - 1) );

    % ----- Ley de control: uso de parámetros estimados (Aest) -----
    % se usa la fila anterior de Aest (estimación en tiempo i-1)
    u(i) = Aest(i-1, 1:np) * phi(1:np);

    % ----- Preparación del vector x para la etapa de identificación -----
    % x contiene los mismos términos que phi pero con desplazamiento por nk
    x = zeros(np,1);
    % primer componente (puede contener mezcla de y y término uf)
    % proteger índices para uf(i-nk)
    uf_in = 0;
    if (i - nk) >= 1
        uf_in = uf(i - nk);
    end
    x(1) = y(i) + lambda / B(1) * uf_in;
    % siguientes componentes: salidas retrasadas
    x(2:na) = -flipud( y(i - na + 2 - nk : i - nk) );
    % controles retrasados (considerando nk desplazamiento)
    x(na+1 : na+nb+nk-2) = -flipud( u(i - nb - nk + 2 - nk : i - 1 - nk) );
    % uf retrasado
    x(na+nb+nk-1 : na+nb+nk+length(P)-3) = -flipud( uf(i - length(P) + 1 - nk : i - 1 - nk) );

    % ----- Identificación recursiva tipo RLS (con olvido) -----
    % estimación de salida y error en identificador
    yh = x' * Aest(i-1, 1:np)';          % salida estimada por identificador en tiempo i-1
    epsi = u(i - nk) - yh;               % error de identificación (u medido menos estimado)

    % Ganancia de Kalman / RLS
    K = (Pmat * x) / (lam + x' * Pmat * x);
    % actualización de covarianza
    Pmat = (Pmat - K * x' * Pmat) / lam;
    % actualización de parámetros (fila i de Aest)
    Aest(i, 1:np) = (Aest(i-1, 1:np)' + K * epsi)';

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