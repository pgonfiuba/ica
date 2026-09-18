%% A. Por qué usar ventanas??
% Buena pregunta! cerremos todas las ventanas y de paso borremos las
% variables
close all 
clear all

%% 1. Configuración de la Señal (Senoide pura con "fuga forzada")
Fs = 1000;                     % Frecuencia de muestreo (Hz)
N = 512;                       % Usamos un bloque corto para ver los bins individuales
t = (0:N-1)/Fs;                % Vector de tiempo

% Forzamos fuga espectral: 150 Hz cae exacto en un bin, 155 Hz NO.
% Al usar 155 Hz, la señal no termina donde empezó al final de las 512 muestras.
f0 = 155;                      
x = sin(2*pi*f0*t);            % Senoide pura sin ruido

%% 2. FFT Sin Ventana vs Con Ventana (Ambas PSDs Compensadas)

% --- Caso A: Sin Ventana (Rectangular) ---
X_sin = fft(x);
P_sin = (2 * abs(X_sin).^2) / (Fs/N);
P_sin = P_sin(1:N/2+1);
f = (0:N/2)*(Fs/N);

% --- Caso B: Con Ventana Hanning 
w = hanning(N);
x_ventaneada = x .* w';        % Multiplicación en el tiempo (w' para transponer si es fila)
X_con = fft(x_ventaneada);
%S_w = sum(w.^2);               % Compensación de energía por la ventana
P_con = (2 * abs(X_con).^2) / (Fs/N);
P_con = P_con(1:N/2+1);

%% 3. Gráficos Comparativos
% Graficamos en escala logarítmica (dB) para ver el "piso" falso que genera la fuga
semilogx(f, 10*log10(P_sin), 'r', 'LineWidth', 1); hold on;
semilogx(f, 10*log10(P_con), 'b', 'LineWidth', 1);
title(['Demostración de Fuga Espectral (Senoide Pura)']);
xlabel('Frecuencia (Hz)');
ylabel('Densidad Espectral de Potencia (dB/Hz)');
grid on; 
xlim([50 250]); % Hacemos zoom alrededor de la zona de interés
legend('Sin Ventana (Fuga Espectral masiva)', 'Con Ventana Hamming (Fuga Controlada)', 'Location', 'southwest');




%% B Qué pasa cuando tenemos una señal estocástica
%1. Configuración de la Señal Filtrada (10,000 muestras)
randn('state',0);
n = 10000;
e = randn(n,1);

Fs = 100;                      % Frecuencia de muestreo (Hz)
t = (0:9999)/Fs;               % Vector de tiempo

A = [1 -1.5 0.7];
B = [0 1 0.5];
x = filter(B,A,e);             % Señal filtrada

%% 2. Cálculos de los Escenarios

% --- Escenario 1: FFT Total Sin Ventana (Rectangular) ---
X_sin = fft(x);
N = length(x);
% PSD Unilateral estándar
P_sin = (2 * abs(X_sin).^2) / (Fs/N);
P_sin = P_sin(1:N/2+1);
f = (0:N/2)*(Fs/N);

% --- Escenario 2: FFT Total Con Ventana Hamming (SIN COMPENSAR) ---
w = hamming(N);
x_ventaneada = x .* w;         % Multiplicación en el tiempo
X_con_sin_comp = fft(x_ventaneada);
% Si usamos la fórmula estándar de ruido blanco sin ajustar por la ventana:
P_con_sin_comp = (2 * abs(X_con_sin_comp).^2) / (Fs/N);
P_con_sin_comp = P_con_sin_comp(1:N/2+1);

% --- Escenario 3: FFT Total Con Ventana Hamming (COMPENSADA) ---
% Calculamos la potencia de la ventana para normalizar correctamente la PSD
S_w = sum(w.^2); 
X_con_comp = fft(x_ventaneada);
% Reemplazamos (N * Fs) por (S_w * Fs) para recuperar la energía perdida
P_con_comp = (2 * abs(X_con_comp).^2) / (S_w * (Fs/N));
P_con_comp = P_con_comp(1:N/2+1);


%% 3. Gráficos Comparativos
figure();

% Gráfico superior: Comparativa espectral directa
subplot(2,1,1);
semilogx(f, 10*log10(P_sin),'r', 'LineWidth', 1); hold on;
semilogx(f, 10*log10(P_con_sin_comp), 'b', 'LineWidth', 1);
title('Efecto de la Ventana de Hamming en la FFT');
ylabel('\phi (dB/Hz)');
legend('Sin Ventana (Rectangular)', 'Con Hamming (SIN COMPENSAR)', 'Location', 'southwest');
grid on; xlim([1 Fs/2]);

% Gráfico inferior: Zoom en bajas frecuencias para ver el sesgo
subplot(2,1,2);
semilogx(f, 10*log10(P_sin), 'r', 'LineWidth', 1); hold on;
semilogx(f, 10*log10(P_con_sin_comp), 'b', 'LineWidth', 1);
title('Zoom en Bajas Frecuencias (Detalle de la caída de energía)');
xlabel('Frecuencia (Hz)'); ylabel('\phi (dB/Hz)');
grid on; xlim([1 10]); % Zoom de 1 a 10 Hz
