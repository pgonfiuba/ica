%% Filtrado de Ruido Blanco
% 
%%  
clear
close all
%%
randn('state',0);
n=10000;
e=randn(n,1);
T0=1;
p1=.9;
A1=poly([p1]);
B1=[0 (1-p1)];
S1=tf(B1,A1,T0);
y1=filter(B1,A1,e);

p2=.99;
A2=poly([p2]);
B2=[0 (1-p2)];
S2=tf(B2,A2,T0);
y2=filter(B2,A2,1*e);
%%
% 
%  Señales temporales
% 
plot([e y1 y2]);grid
axis([2000 2500 -4 4])
title(' Señales Temporales')
legend('e','y1','y2')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
%%
% 
%  Correlaciones
% 
xe=xcorr(e,'coeff');
xy1=xcorr(y1,'coeff');
xy2=xcorr(y2,'coeff');
plot([xe xy1 xy2],'LineWidth',4);grid
delta=100;
axis([n-delta n+delta -.2 1])
title('Correlaciones')
legend('e','y1','y2')

%%
% 
%  Espectros
% 
w = logspace(-2,pi,128/4)';
pe=spa(e,[],w);
py1=spa(y1,[],w);
py2=spa(y2,[],w);
%bodeplot(pe,py1,py2);grid
bode(pe,py1,py2);grid
title('Espectros')
legend('e','y1','y2')
h = findobj(gcf,'type','line');
set(h,'linewidth',4);
