
clear; clc; close all;

%% Configuracion del ensayo

Texp = 360;          % Duracion [s]
Ts_adq = 0.05;      % Telemetria Wemos: 20 Hz
Ts_prbs = 0.05;      % Actualizacion PRBS: 20 Hz
A = 1;              % Amplitud de la entrada 

%% Generar PRBS
% Requiere System Identification Toolbox
N = ceil(Texp/Ts_prbs) + 1;
prbs = A * idinput(N, 'prbs', [0 1], [-1 1]);
prbs = prbs(:);

%prbs = ones(N,1);

%% Abrir puerto serie
s = serial('/dev/ttyUSB0', ...
    'BaudRate', 115200, ...
    'Terminator', 'LF', ...
    'Timeout', 2);

fopen(s);
pause(2);
flushinput(s);

fprintf(s, '0\n');


% El puerto puede reiniciar el ESP8266 al abrirse
pause(2);
flushinput(s);
fprintf(s, '0\n');

fprintf("Esperando telemetria del Wemos...\n");

%% Esperar una trama valida
while true
    linea = fgetl(s);
    v = sscanf(linea, '%f');

    if numel(v) == 3 && all(isfinite(v))
        break;
    end
end

fprintf("Adquisicion iniciada.\n");

%% Adquisicion
% Columnas recibidas: timestamp_us, q, u
T = zeros(0,3);

t0 = tic;
t_cambio = 0;
k = 1;

% Aplicar el primer nivel de la PRBS
fprintf(s, '%.3f\n', prbs(k));
k = k + 1;
t_cambio = Ts_prbs;

% Guardar la primera trama valida
T(end+1,:) = v(:)';

while toc(t0) < Texp

    % Leer una trama del Wemos
    linea = fgetl(s);
    v = sscanf(linea, '%f');

    if numel(v) ~= 3 || any(~isfinite(v))
        continue;
    end

    % Guardar timestamp, posicion y entrada realmente aplicada
    T(end+1,:) = v(:)';

    % Actualizar la PRBS segun el reloj de MATLAB.
    % El Wemos aplicara el valor cuando procese el comando.
    t = toc(t0);

    if t >= t_cambio
        if k <= numel(prbs)
            fprintf(s, '%.3f\n', prbs(k));
            k = k + 1;
        else
            fprintf(s, '0\n');
        end

        t_cambio = t_cambio + Ts_prbs;
    end
end

% Dejar la entrada en cero al terminar
fprintf(s, '0\n');

fclose(s);
delete(s);
clear s;

%% Procesamiento de los datos

timestamp_us = T(:,1);
q = T(:,2);
u = T(:,3);

% Tiempo relativo usando el reloj del Wemos.
% Contempla el desbordamiento de micros() a los ~71.6 min.
dt_us = mod(diff(timestamp_us), 2^32);
t = [0; cumsum(dt_us)] * 1e-6;

datos = table(t, timestamp_us, q, u);

%% Graficos
figure;

subplot(2,1,1);
plot(t, q);
grid on;
ylabel('q [rad]');
title('Respuesta de la planta');

subplot(2,1,2);
stairs(t, u);
grid on;
ylabel('u [V]');
xlabel('Tiempo [s]');
title('Entrada realmente aplicada');

%% Guardar
%writetable(datos, 'datos_plata_prbs.csv');
%save('datos_plata_prbs.mat', 'datos', 'prbs', 'Ts_adq', 'Ts_prbs', 'A');
save('datos_plata.mat', 'datos');

fprintf('\nMuestras adquiridas: %d\n', height(datos));
fprintf('Duracion registrada: %.3f s\n', t(end));
fprintf('Intervalo medio: %.3f ms\n', 1000*mean(diff(t)));
fprintf('Datos guardados.\n');