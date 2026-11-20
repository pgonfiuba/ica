
clear all; close all; clc

%% 1. Parámetros de simulación
rng(1234);

cambioref = 50;
ciclos = 5;
n = cambioref*ciclos;

e = 0.02*randn(n,1);
y = zeros(n,1);
u = zeros(n,1);

k_cambio = 120;


%% 2. Modelo de la planta

% A(q^-1) y(k) = q^-d B'(q^-1) u(k) + C(q^-1) e(k)

A = [1 -1.74 0.77];
B = [0.0011 0.0009]*1000;

na = length(A);
nb = length(B);
nk = 2;

% Modelo del ruido
C = [1 -2 1.29 -0.27];
nc = length(C);


%% 3. Modelo de referencia

% M(q^-1) = Bm(q^-1)/Am(q^-1)

Bm = 0.4;
Am = [1 -0.6 0 0 0];

nBm = length(Bm);
nAm = 2;

Ra = zeros(n,1);
RaC = zeros(n,1);
ym = zeros(n,1);


%% 4. Señal de referencia

ref = ones(cambioref,1);

for i = 1:ciclos-1
    ref = [ref; (-1)^i*ones(cambioref,1)];
end


%% 5. Cálculo de F y G

% Identidad Diofantina:
%
% C*Am = F*A + q^-nk*G

CAm = conv(C,Am(1:nAm));

% Polinomio F
F = zeros(nk,1);
F(1) = 1;

for i = 2:nk

    F(i) = CAm(i);

    for j = 1:i-1
        if i-j+1 <= length(A)
            F(i) = F(i) - F(j)*A(i-j+1);
        end
    end

end


% Polinomio G

FA = conv(F,A);

L = max(length(CAm),length(FA));

residuo = zeros(L,1);

residuo(1:length(CAm)) = residuo(1:length(CAm)) + CAm(:);
residuo(1:length(FA)) = residuo(1:length(FA)) - FA(:);

G = residuo(nk+1:end);

%% 6. Cálculo de los parámetros del controlador

% Producto de polinomios F*B'
FB = conv(F,B(:));

% Producto C*Bm
CBm = conv(C(:),Bm(:));

% Ley de control:
%
% FB'*u(k) = CBm*r(k) - G*y(k)

% Parámetros fijos del controlador
b0 = FB(1);

ParReg = [1/b0;
          G/b0;
          FB(2:end)/b0];


%% 7. Simulación en lazo cerrado

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
    % Referencia del controlador
    % ------------------------------------------

    % RaC = C*Bm*r

    for j = 1:length(CBm)
        if k-j+1 > 0
            RaC(k) = RaC(k) + CBm(j)*ref(k-j+1);
        end
    end


    % ------------------------------------------
    % Vector de regresión del controlador
    % ------------------------------------------

    phi_reg = zeros(length(ParReg),1);

    % Referencia filtrada
    phi_reg(1) = RaC(k);

    % Salidas pasadas (incluye y(k))
    phi_reg(2:1+length(G)) = ...
        -y(k:-1:k-length(G)+1);

    % Entradas pasadas
    phi_reg(2+length(G):end) = ...
        -u(k-1:-1:k-length(FB)+1);


    % ------------------------------------------
    % Ley de control
    % ------------------------------------------

    u(k) = phi_reg'*ParReg;

end


%% 8. Resultados

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
