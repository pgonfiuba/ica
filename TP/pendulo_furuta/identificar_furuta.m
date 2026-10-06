%% Planta
% Vamos solo con la dinámica del péndulo, entrando con tau1 y saliendo con
% el angulo del eje 2

Ts = 1e-2;

% Dinámica de la planta en el punto de equilibrio
% (sin controlador PD ni saturaciones)
G = (-878.7 * s - 9.258e-11)/(s^3 + 746.6 *s^2 + 986.5* s + 4.023e04);
Gd = c2d(G,Ts,'zoh');


%% Simulación
close all

N = 30000;
u = sign(randn(N,1));

% ruido
e = 0.01*randn(N,1);

y = lsim(Gd,u);
ym = y + e;
t = (0:N-1)*Ts;

figure()
subplot(2,1,1)
plot(t,ym,'Linewidth',1.5)
ylabel('y')
subplot(2,1,2)
plot(t,u,'Linewidth',1.5)
ylabel('u')
xlabel('Tiempo (s)')

%% Estimacion de la rta al impulso por correlacion
% Correlación entrada-salida
m = 1000;
R = covf([ym u],m+1);

% Estimación de la respuesta al impulso
h = R(2,:)'/R(4,1);

% Ojo con esto:
% In discrete time, impulse computes the response to a unit-area 
%    pulse of length Ts and height 1/Ts where Ts is the sample time
% => debo multiplicar por Ts para comparar
y_imp = impulse(Gd,m*Ts)*Ts;

% Comparación con la respuesta al impulso del modelo
figure
plot([y_imp(1:m) h(1:m)],'LineWidth',1.5)
grid
legend('Respuesta al impulso','Estimación por correlación')



%% Identificación

z = iddata(ym,u,Ts);
%m_param = iv4(z,[3 3 1])
m_param = armax(z,[3 3 4 1])

w = logspace(0,2,100);
m_espectral = etfe(z)

figure()
bode(w,G,m_param,m_espectral)
grid on
h = findobj(gcf,'type','line'); set(h,'linewidth',1.5);
legend('Planta linealizada','Paramétrico','Espectral')

%%
figure
resid(z,m_param)

%%
figure
pzmap(Gd, m_param)
legend('Planta discretizada','Paramétrico')