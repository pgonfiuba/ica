%% 0. Planta y experimento

clear; close all; clc

Ts = 1;

% Planta real (solo conocida para generar los datos)
G = tf([0.8 0.3],[1 -1.3 0.42],Ts);

N = 1000;

% Excitación para identificación
u = idinput(N,'prbs');

% Salida del sistema + ruido
y0 = lsim(G,u);
y  = y0 + 0.05*randn(N,1);

t = (0:N-1)'*Ts;

figure
plot(t,u,t,y)
legend('u','y')
grid on
xlabel('Tiempo')


%% 1. Separación de los datos

Nid = floor(0.6*N);

z_id  = iddata(y(1:Nid),u(1:Nid),Ts);
z_val = iddata(y(Nid+1:end),u(Nid+1:end),Ts);

figure
plot(z_id)
title('Datos de identificación')

figure
plot(z_val)
title('Datos de validación')

%% Set de validación adicional con una entrada de interés: escalón

Nval = 400;
u_step = ones(Nval,1);
u_step(1:20) = 0;       % escalón después de un breve reposo

% Sistema real
y_step = lsim(G,u_step);

% Agregar ruido de medición
y_step = y_step + 0.05*randn(Nval,1);

% Datos de validación
z_step = iddata(y_step,u_step,Ts);

figure
plot(z_step)
title('Datos de validación para entrada escalon')

%% 2. ETFE: análisis frecuencial no paramétrico

G_etfe = etfe(z_id);

figure
bode(G_etfe)
grid on
title('ETFE - identificación')

% Esto es hacer trampa porque no conocemos la "planta real"
%figure
%bode(G_etfe)
%hold on
%bode(G)
%grid on
%legend('ETFE','Planta real')

%% 3. Selección de estructura ARX

NN = struc(2:10,2:10,1:3);

V = arxstruc(z_id,z_val,NN);

nn = selstruc(V,'plot');

%% Modelo ARX seleccionado

m_arx = arx(z_id,nn);

% Muestro el modelo identificado
m_arx

%% 4. Polos y ceros del modelo

figure
pzmap(m_arx)
grid on
title('Polos y ceros del modelo ARX')

% Distancia entre polos y ceros

p = pole(m_arx);
z = zero(m_arx);

for i = 1:length(p)
    d = abs(z-p(i));
    fprintf('Polo %.3f%+.3fj - distancia al cero más cercano: %.4f\n', ...
        p(i),imag(p(i)),min(d));
end

%% 5. Comparación frecuencial

G_idfrd = idfrd(m_arx);

figure
bode(G_etfe,G_idfrd)
grid on
legend('ETFE','ARX')
title('Modelo paramétrico vs. estimación no paramétrica')

%% 6. Análisis de residuos

figure
resid(z_val,m_arx)

%% 7. Validación en simulación
% Vamos con validación en simulación. Para predicción poner el horizonte
% horizonte=1;
% compare(z_id,m_arx,horizonte);

figure
compare(z_id,m_arx)
grid on
title('Ajuste sobre datos de identificación')

figure
compare(z_val,m_arx)
grid on
title('Validación sobre datos nuevos')

figure
compare(z_step,m_arx)
grid on
title('Validación sobre escalon')

%% 8. Solo para verificar el resultado

figure
bode(G,m_arx)
grid on
legend('Planta real','Modelo identificado')
title('Modelo identificado vs. planta real')
