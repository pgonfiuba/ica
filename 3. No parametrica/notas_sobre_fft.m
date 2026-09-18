%% 1. Configuración de la Señal (10,000 muestras)
randn('state',0);
n=10000;
e=randn(n,1);

Fs = 100;                     % Frecuencia de muestreo (Hz)
t = (0:9999)/Fs;               % Vector de tiempo (10,000 muestras)

A = [1 -1.5 0.7];
B = [0 1 0.5];

S = tf(B,A,1/Fs);
x = filter(B,A,e);

%% 2. Cálculos de los 3 Escenarios

% --- Escenario A: fft(x, 512) ---
% Solo toma las primeras 512 muestras.
X_recortada = fft(x, 512);
P_recortada = (abs(X_recortada)/512).^2; % Potencia de la FFT
P_recortada = P_recortada(1:257);        % Nos quedamos con la mitad positiva
% Cada bin es de ancho Deltaf=Fs/512
% Entonces para explicar realmente la potencia que entró en el bin tenemos
% que dividir por Deltaf
% Además tenemos que multiplicar por 2 porque vemos la versión unilateral
P_recortada = P_recortada(1:257)/(Fs/512)*2;        
f_512 = (0:256)*(Fs/512);

% --- Escenario B: fft(x) con el Total (10,000 muestras) ---
% Toma toda la señal, pero al no promediar, el piso de ruido oscila salvajemente.
X_total = fft(x);
N_total = length(x);
% Cuidado que cada bin es de ancho Deltaf=Fs/N_total
% Entonces para explicar realmente la potencia que entró en el bin tenemos
% que dividir por Deltaf
% Además tenemos que multiplicar por 2 porque vemos la versión unilateral
P_total = 2 * (abs(X_total)/N_total).^2 / (Fs/N_total);
P_total = P_total(1:N_total/2+1);
f_total = (0:N_total/2)*(Fs/N_total);

% --- Escenario C: Welch con ventanas de 512 (Promediado) ---
% Divide en bloques de 512, promedia (mitiga ruido), pero la ventana de Hamming 
% ensancha la base del pico (pérdida de resolución/resolución espectral).
[P_welch, f_welch] = pwelch(x, 512, [], 512, Fs);


%% 3. Gráficos Comparativos
figure();

% Gráfico 1: FFT Recortada
%subplot(3,1,1);
semilogx(f_total, 10*log10(P_total), 'b', 'LineWidth', 0.8);
hold on
semilogx(f_512, 10*log10(P_recortada), 'r', 'LineWidth', 1);
semilogx(f_welch, 10*log10(P_welch), 'g', 'LineWidth', 1.5);
xlabel('Frecuencia (Hz)'); ylabel('Potencia (dB)'); grid on; xlim([1 Fs/2]);
legend('fft completa','fft-512','Welch')