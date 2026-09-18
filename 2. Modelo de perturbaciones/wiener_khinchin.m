%% 1. Configuración de la Señal (Ejemplo con el ruido filtrado anterior)
randn('state',0);
n = 10000;
e = randn(n,1);
Fs = 100;                      % Frecuencia de muestreo (Hz)

A = [1 -1.5 0.7];
B = [0 1 0.5];
y1 = filter(B,A,e);            

%% 2. Camino A: Wiener-Khinchin Manual (FFT de la Autocorrelación Ventaneada)
% Usamos 'biased' para que la escala de la potencia sea físicamente correcta
[R, lags] = xcorr(y1, 'biased'); 

NFFT=256
% Buscamos el centro (donde lag = 0)
centro = find(lags == 0);
M = NFFT/2; % Longitud de la ventana a cada lado (Equivale a un ancho total de ~256)

% Recortamos el trozo central de la autocorrelación (-M a +M)
R_recortada = R(centro - M : centro + M);
N_r = length(R_recortada);     % N_r será 257 muestras

% Aplicamos una ventana de Hamming para suavizar el espectro y evitar fuga
w_hamming = hamming(N_r);
R_ventaneada = R_recortada .* w_hamming;

% FFT de la autocorrelación ventaneada (Desplazamos para que el centro temporal sea 0)
% Al estar la señal en el centro, usamos fftshift antes de la FFT
X_corr = fft(fftshift(R_ventaneada));

% Convertimos a PSD Bilateral
P_wiener = abs(real(X_corr)) / Fs; 
P_wiener = P_wiener(1:floor(N_r/2)+1);
f_wiener = (0:floor(N_r/2)) * (Fs / N_r);

%% 3. Camino B: Uso de spa (System Identification Toolbox)
% spa recibe un objeto iddata. w representa el tamaño de la ventana de lags (M).
data = iddata(y1, [], 1/Fs);
g_spa = spa(data, M);          % M especifica el ancho de banda del ventaneo de lags

% Extraemos la PSD calculada por spa
[P_spa, w_spa] = spectrum(g_spa);
P_spa = squeeze(P_spa);        % Elimina dimensiones vacías de la matriz de SPA
f_spa = w_spa / (2*pi);        % Convertimos rad/s a Hz

%% 4. Gráficos Comparativos
figure()

semilogx(f_wiener, 10*log10(P_wiener), 'r', 'LineWidth', 2); hold on;
semilogx(f_spa, 10*log10(P_spa), 'b', 'LineWidth', 2);

title('Verificación del Teorema de Wiener-Khinchin: FFT(R_{xx}) vs SPA');
xlabel('Frecuencia (Hz)');
ylabel('Densidad Espectral de Potencia (dB/Hz)');
grid on; xlim([1 Fs/2]);
legend('FFT de xcorr (Ventaneada)', 'spa(y1)', 'Location', 'southwest');
