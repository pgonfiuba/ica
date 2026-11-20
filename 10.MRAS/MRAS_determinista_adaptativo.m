clear all; close all; clc
%% 1. Parámetros de simulación

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

% M(q^-1) = Bm(q^-1)/Am(q^-1)

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

% Parámetros del RLS
lambda_f = 0.99;              % Factor de olvido
Cov = 1000*eye(length(ParReg));

ParReg_ad = ParReg;           % Estimación inicial

%% 7. Simulación en lazo cerrado

ParHist = zeros(length(ParReg_ad),n);
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

    % Referencia filtrada: Ra = H*r
    for j = 1:nBm
        Ra(k) = Ra(k) + Bm(j)*ref(k-j+1);
    end

    % Dinámica del modelo: Am*ym = Ra
    ym(k) = Ra(k);

    for j = 2:nAm
        ym(k) = ym(k) - Am(j)*ym(k-j+1);
    end

    
    % ------------------------------------------
    % Identificación recursiva de los parámetros
    % ------------------------------------------

    if k > nk+nb+5

        j = k-nk;

        % Variable auxiliar: ya(k+d) = Am*y(k+d)
        ya = 0;

        for m = 1:nAm
            ya = ya + Am(m)*y(k-m+1);
        end

        % Regresor de la planta
        phi_id = zeros(length(ParReg_ad),1);
        phi_id(1) = ya;
        phi_id(2:na) = -flipud(y(j-na+2:j));
        phi_id(na+1:end) = -flipud(u(j-nb-nk+2:j-1));

        % Actualización RLS
        K = Cov*phi_id / (lambda_f + phi_id'*Cov*phi_id);
        errorID = u(j) - phi_id'*ParReg_ad;
        ParReg_ad = ParReg_ad + K*errorID;
        Cov = (Cov - K*phi_id'*Cov)/lambda_f;
        ParHist(:,k) = ParReg_ad;
    end


    % ------------------------------------------
    % Vector de regresión del controlador
    % ------------------------------------------

    phi_reg = zeros(na+nb+nk-2,1);

    % Referencia filtrada
    phi_reg(1) = Ra(k);

    % Salidas pasadas
    phi_reg(2:na) = -flipud(y(k-na+2:k));

    % Entradas pasadas
    phi_reg(na+1:na+nb+nk-2) = -flipud(u(k-nb-nk+2:k-1));


    % ------------------------------------------
    % Ley de control
    % ------------------------------------------
    
    u(k) = phi_reg'*ParReg_ad;

end


%% 8. Resultados
t = 0:length(ym)-1;

figure;
subplot(2,1,1)
plot(t,y,'b','LineWidth',1.5);
hold on;
plot(t(nk+1:end),ym(1:length(ym)-nk),'k--','LineWidth',1.2);
axis([0 n -1.5 1.5]);
grid on;
legend('y','y^M','Location','southeast');
ylabel('y');

subplot(2,1,2)
stairs(u,'r');

grid on;
xlabel('Muestra');
ylabel('u');

%% Muestro la evolución de los parámetros
figure;

plot(ParHist','LineWidth',1.5);
yl = ylim;
line([k_cambio k_cambio], yl,'Color','k','LineStyle','--');
text(k_cambio, yl(2), ' Cambio de planta','VerticalAlignment','top');
grid on;

xlabel('Muestra');
ylabel('Parámetros estimados');
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5');

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