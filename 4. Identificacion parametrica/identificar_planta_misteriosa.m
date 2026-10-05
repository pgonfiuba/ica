clc; close all; clear all;
load 'data_misteriosa.mat'

u = data(:,1);
y = data(:,2);
Ts = 0.1
N = length(u);

Nid = floor(0.6*N);

z_id  = iddata(y(1:Nid),u(1:Nid),Ts);
z_val = iddata(y(Nid+1:end),u(Nid+1:end),Ts);

%%
G_spa = spa(z_id);
G_etfe = etfe(z_id);

figure
bode(G_etfe,G_spa)
legend('etfe','spa')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on
title('Identificación espectral')


%% Si queremos filtrar (OPCIONAL)
wc = 30;                    
Wn = wc*Ts/pi;

[b,a] = butter(4,Wn,'low');

yf = filtfilt(b,a,y);
uf = filtfilt(b,a,u);

z_id_f  = iddata(yf(1:Nid),uf(1:Nid),Ts);
z_val_f = iddata(yf(Nid+1:end),uf(Nid+1:end),Ts);

G_etfe_f = etfe(z_id_f);

figure
bode(G_spa,G_etfe,G_etfe_f)
legend('spa','etfe sin filtrar','etfe filtrado')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on
title('SPA - identificación')


%%
close all

% Evaluar qué ocurre con estos 2 métodos
G_pol = iv4(z_id,[4 4 1])
%G_pol = armax(z_id,[4 4 4 1])

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

%% Corroborar el modelo contra la planta
open('planta_misteriosa')
sim('planta_misteriosa')
