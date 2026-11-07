%% Ejemplo de identificación a partir de los datos capturados

clc; close all; %clear all;
load 'datos_planta.mat'

u = datos.u;
y = datos.q;
t = datos.t;
Ts = mean(diff(t))
N = length(u);

z_id  = iddata(y,u,Ts);

%% Identificación no paramétrica
G_spa = spa(z_id);
G_etfe = etfe(z_id);

figure
bode(G_etfe,G_spa)
legend('etfe','spa')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on
title('Identificación espectral')


%% Indetificación paramétrica (ajustar los ordenes y el tipo de modelo)
close all

% Evaluar qué ocurre con estos 2 métodos
%G_pol = iv4(z_id,[4 4 1])
G_pol = armax(z_id,[4 4 4 1])

figure()
bode(G_spa,G_etfe,G_pol)
legend('spa','etfe','iv4')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on

figure
pzmap(G_pol)

figure
resid(z_id,G_pol)

