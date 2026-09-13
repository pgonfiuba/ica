%%
clear all
close all

%%
z = tf('z',1);
H = 1/(z-0.9)

randn('state',0);
n = 10000;
e = randn(n,1);
t = (0:n-1)';

x = lsim(H,e,t);

r_ee = xcorr(e,'coeff');
[r_xx, lags] = xcorr(x,'coeff');

figure()
plot(lags,[r_ee r_xx],'LineWidth',4);grid
axis([-100 100 -.2 1])
title('Autocorrelaciones')
legend('e','x')

%%
w = logspace(-2,log10(pi),32)';
phi_ee = spa(e,100,w);
phi_xx = spa(x,100,w);
fig_bode = figure();
bode(phi_ee,phi_xx); grid
title('Espectros')
legend('\phi_e','\phi_x')
h = findobj(gcf,'type','line');
set(h,'linewidth',4);

%%
% Estimación del polo del sistema a partir de una realización
% Magia negra
[a_hat, sigma_e_hat] = aryule(x,1);

%% Factorización espectral a partir de la realización
a1 = -a_hat(2);

% Polos recíprocos del espectro
p = [a1; 1/a1];

figure()
zplane([],p)
grid
title('Polos del espectro estimado')

%%
H_hat = 1/(z-a1);
H_inv = 1/(1/z-a1);

Phi_hat = H_hat*H_inv;

[magH,~,wH] = bode(Phi_hat,w);
magH = squeeze(magH);

[magPhi,~,~] = bode(phi_xx,w);
magPhi = squeeze(magPhi);

figure
loglog(w,magH,'LineWidth',3)
hold on
loglog(w,magPhi,'LineWidth',3)
grid
legend('|H|^2','\Phi_{xx}')
xlabel('\omega')
ylabel('Magnitud')
title('Filtro generador vs. espectro de la realización')