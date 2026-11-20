clear all; close all; clc
%% 1. Parámetros de simulación
rng(1234);

cambioref = 50;         % Duración de cada escalón
ciclos = 5;             % Cantidad de escalones
n = cambioref*ciclos;

e = 0.02*randn(n,1);    % Ruido blanco
y = zeros(n,1);         % Salida de la planta
u = zeros(n,1);         % Señal de control

k_cambio = 120;         % Instante de reducción de ganancia


%% 2. Modelo de la planta

% A(q^-1) y(k) = q^-d B'(q^-1) u(k)

A = [1 -1.74 0.77];
B = [0.0011 0.0009]*1000;

na = length(A);
nb = length(B);
nk = 2;                 % Retardo d

% Modelo del ruido
C = [1 -2 1.29 -0.27];
nc = 4;


%% 3. Modelo de referencia

% M(q^-1) = H(q^-1)/E(q^-1)

Bm = 0.4;
Am = [1 -0.6 0 0 0];

nBm = length(Bm);
nAm = 2;

Ra = zeros(n,1);        % Referencia filtrada
ym = zeros(n,1);        % Salida del modelo de referencia


%% 4. Señal de referencia

ref = ones(cambioref,1);

for i = 1:ciclos-1
    ref = [ref; (-1)^i*ones(cambioref,1)];
end


%% 5. Cálculo de F y G

% Am = F*A + q^-nk*G

% Polinomio F
F = zeros(nk,1);
F(1) = 1;

for i = 2:nk

    F(i) = Am(i);

    for j = 1:i-1
        if i-j+1 <= length(A)
            F(i) = F(i) - F(j)*A(i-j+1);
        end
    end

end


% Polinomio G
G = zeros(na,1);

for i = 1:na

    G(i) = Am(nk+i);

    for j = 0:nk-1
        if i+nk-j <= length(A)
            G(i) = G(i) - F(j+1)*A(i+nk-j);
        end
    end

end


%% 6. Cálculo de los parámetros del controlador

% Producto de polinomios F*B'
FB = conv(F,B)';

% Ley de control:
%
% u(k) = 1/b0 * [ Ra(k) - G*y(k) + (b0-FB)*u(k) ]
%
% Expresión vectorial:
%
% u(k) = ValReg' * ParReg

b0 = B(1);

% Vector de parámetros del controlador
ParReg = zeros(na+nb+nk-2,1);

ParReg(1) = 1/b0;

% Coeficientes asociados a las salidas pasadas
ParReg(2:na) = G(1:na-1)/b0;

% Coeficientes asociados a las entradas pasadas
ParReg(na+1:na+nb+nk-2) = FB(2:nb+nk-1)/b0;


%% 7. Simulación en lazo cerrado

for k = nk+nb+5:n

    % ------------------------------------------
    % Cambio en la planta
    % ------------------------------------------

    if k == k_cambio
        B = B/2;
    end


    % ------------------------------------------
    % Planta
    % ------------------------------------------

    % A*y(k) = q^-nk * B' * u(k)

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
        y(k) = y(k) + C(j)*e(k-j);
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
    % Vector de regresión del controlador
    % ------------------------------------------

    ValReg = zeros(na+nb+nk-2,1);

    % Referencia filtrada
    ValReg(1) = Ra(k);

    % Salidas pasadas
    ValReg(2:na) = -flipud(y(k-na+2:k));

    % Entradas pasadas
    ValReg(na+1:na+nb+nk-2) = ...
        -flipud(u(k-nb-nk+2:k-1));


    % ------------------------------------------
    % Ley de control
    % ------------------------------------------

    u(k) = ValReg'*ParReg;

end


%% 8. Resultados

t = 0:length(ym)-1;

figure;
subplot(2,1,1)
plot(t,y,'b','LineWidth',1.5);
hold on;
plot(t(nk+1:end),ym(1:length(ym)-nk),'k--','LineWidth',1.2);
grid on;
axis([0 n -1.5 1.5]);
legend('y','y^m','Location','southeast');
ylabel('y');

subplot(2,1,2)
stairs(u,'r');

grid on;
xlabel('Muestra');
ylabel('u');

%% Error de seguimiento filtrado
yf = filter(Am(1:nAm),1,y);
ymf = filter(Am(1:nAm),1,ym);

error_filtrado = yf-ymf;

figure;
plot(error_filtrado);
grid on;
xlabel('Muestra');
ylabel('Error filtrado');

% Vemos la varianza del error filtrado en un intervalo de tiempo donde la
% referencia está en 0
var(error_filtrado(60:95))