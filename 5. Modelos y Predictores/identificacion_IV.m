%% Identificación por IV
% Ejemplo de identificación en Lazo Cerrado de un sistema con un integrador
% puro usando IV

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
thetaTrue = [-th.A(2:end) th.B(2:end)]';
disp('Modelo "verdadero":');
disp(thetaTrue');

% Simulación en LC con un control Kp=1
Kp = 1;
N = 30000;

% Ruido blanco de medición
rb = 0.2*randn(N,1);

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

%% Registro auxiliar: estimacion preliminar por LS
% Vamos con un modelo ARX221 por mínimos cuadrados -> sabemos que va a dar
% sesgado 
% Trabajamos sobre un set diferente de muestras
N1 = floor(0.3*N);      % Registro para el modelo auxiliar
N2 = N-N1;          % Registro para evaluar LS e IV
k = (5:N1)';

Y = ym(k);
Phi = [ym(k-1), ym(k-2), u(k-1), u(k-2)];
       
% Modelo auxiliar ARX estimado por minimos cuadrados
thetaAux = Phi \ Y;

disp('Parametros del modelo auxiliar (LS):');
disp(thetaAux');

%% IV variante 1: retardos de la referencia
% Cuatro parametros -> cuatro instrumentos
k = 3:N2;
Zref = [r(N1+k-1), r(N1+k-2), r(N1+k-3), r(N1+k-4)];
Y = ym(N1+k);
Phi = [ym(N1+k-1), ym(N1+k-2), u(N1+k-1), u(N1+k-2)];
thetaIVref = (Zref' * Phi) \ (Zref' * Y);

disp('Parametros del modelo por IV (caso naive):');
disp(thetaIVref');

%% IV variante 2: instrumentos generados por el modelo auxiliar
% Simulacion sin ruido del modelo auxiliar en lazo cerrado.
u_sim  = zeros(N2,1);
y_sim  = zeros(N2,1);
for kk = 3:N2
    % Calculo la acción de control "auxiliar"
    u_sim(kk-1)  = Kp*(r(kk-1+N1) - y_sim(kk-1));    
    % Calculo la evolución del sistema "auxiliar"
    y_sim(kk) = [y_sim(kk-1) y_sim(kk-2) u_sim(kk-1) u_sim(kk-2)] * thetaIVref;
end

% Reemplazamos los regresores medidos por sus versiones simuladas.
Zmod = [y_sim(k-1) y_sim(k-2) u_sim(k-1) u_sim(k-2)];
thetaIVmod = (Zmod' * Phi) \ (Zmod' * Y);

disp('Parametros del modelo por IV (caso 2):');
disp(thetaIVref');

%% Comparacion de estimaciones
Resultados = table( ...
    thetaTrue, thetaAux, thetaIVref, thetaIVmod, ...
    'VariableNames', {'Verdadero', 'LS', ...
                      'IV_referencia', 'IV_modelo'}, ...
    'RowNames', {'a1', 'a2', 'b1', 'b2'});

disp(Resultados);

fprintf('\nCondicion de las matrices instrumentales:\n');
fprintf('IV referencia: %.3g\n', cond(Zref' * Phi));
fprintf('IV modelo:     %.3g\n', cond(Zmod' * Phi));

%% Comparacion grafica de parametros
figure;
bar([thetaTrue, thetaAux, thetaIVref, thetaIVmod]);
grid on;
xticks(1:4);
xticklabels({'a_1','a_2','b_1','b_2'});
ylabel('Valor estimado');
legend('Verdadero','LS','IV referencia','IV modelo', ...
       'Location','best');
title('Identificacion ARX en lazo cerrado');
