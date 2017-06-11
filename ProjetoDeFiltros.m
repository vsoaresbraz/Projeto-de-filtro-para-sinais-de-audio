close all
clear
clc
%% Gravando o áudio
Samp1=audioread('Baixo.wav');
Samp2=audioread('Medio.wav');
Samp3=audioread('Alto.wav');
Samp=Samp1+Samp2+Samp3;
%% Parâmetros
amostra=size(Samp);
N=amostra(1,1);        
fs=16000;              
T=N/fs;                
t=(0:N-1)/N;           
t=t*T;                
%% Transformada de Fourier
X=fft(Samp);
%% Áudio Original
%Áudios de base
figure(1)
subplot(2,2,1);
plot(t,Samp1);
title('Áudio de Baixa Frequência');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
ylabel('Amplitude');
grid;
subplot(2,2,2);
plot(t,Samp2);
title('Áudio de Média Frequência');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
ylabel('Amplitude');
grid;
subplot(2,2,3);
plot(t,Samp3);
title('Áudio de Alta Frequência');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
ylabel('Amplitude');
grid;
%Áudio Composto
subplot(2,2,4);
plot(t,Samp(:,1));
title('Áudio Composto');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
ylabel('Amplitude');
grid;
%% Espectro Anterior a Filtragem
X1=abs(fft(Samp(:,1))/(N/2));
X1=X1(1:N).^2;
f=(0:N-1)/T;
figure
subplot(2,2,1);
plot(f,X1);
title('Espectro Anterior a Filtragem');
ylabel('Energia');
xlabel('Frequência (Hz)');
grid;
axis([300 2e+3 0 1e-4]);
%% Parâmetros do Filtro

% Filtro Elíptico (filtrando as médias)
Wreject=[2 780]*2*pi;
Wpass=[1 781]*2*pi;
Rpass=0.1;
Rstop=30;

[n,wn]=ellipord(Wreject,Wpass,Rpass,Rstop,'s');
[num,den]=ellip(n,Rpass,Rstop,wn,'stop','s');
H=freqs(num,den,f*2*pi);

Wreject1=[1000 4000]*2*pi;
Wpass1=[999 4001]*2*pi;

[nb,wnb]=ellipord(Wreject1,Wpass1,Rpass,Rstop,'s');
[numb,denb]=ellip(nb,Rpass,Rstop,wnb,'stop','s');
H1=freqs(numb,denb,f*2*pi);

H2=H.*H1;

subplot(2,2,3)
plot(f,20*log10(abs(H2)));
xlabel('Frequência (Hz)');
ylabel('Ganho (dB)');
grid;
title('Filtro Passa-Faixas (F. Médias)')
axis([300 2e+3 -100 0]);

%Filtro Elíptico (filtrando as baixas)
wr=765*2*pi;
wp=795*2*pi;
rp=0.1;
rs=30;

[nb,wnb]=ellipord(wr,wp,rp,rs,'s');
[numb,denb]=ellip(nb,rp,rs,wnb,'low','s');
H3=freqs(numb,denb,f*2*pi);

subplot(2,2,2)
plot(f,20*log10(abs(H3)));
xlabel('Frequência (Hz)');
ylabel('Ganho (dB)');
grid;
title('Filtro Passa-Faixas (F. Baixas)')
axis([300 2e+3 -100 0]);

%Filtro Elíptico (filtrando as altas)
Wr=1550*2*pi;
Wp=1400*2*pi;
Rp=0.1;
Rs=30;

[na,wna]=ellipord(Wr,Wp,Rp,Rs,'s');
[numa,dena]=ellip(na,Rp,Rs,wna,'high','s');
H4=freqs(numa,dena,f*2*pi);

subplot(2,2,4)
plot(f,20*log10(abs(H4)));
xlabel('Frequência (Hz)');
ylabel('Ganho (dB)');
grid;
title('Filtro Passa-Faixas (F. Altas)')
axis([300 2e+3 -100 0]);
%% Espectro Posterior a Filtragem
%Espectro Original
figure
subplot(2,2,1);
plot(f,X1);
title('Espectro Anterior a Filtragem');
ylabel('Energia');
xlabel('Frequência (Hz)');
grid;
axis([300 2e+3 0 1e-4]);
% Filtragem das F. Baixas
Y1=X.*H3';
subplot(2,2,2);
Yb=abs(Y1)/(N/2);
Yb=Yb(1:N).^2;
plot(f,Yb);
title('Espectro Posterior a Filtragem (F. Baixas)');
ylabel('Energia');
xlabel('Frequência (Hz)');
grid;
axis([300 2e+3 0 1e-4]);
% Filtragem das F. Médias
Y2=X.*H2';
subplot(2,2,3);
Ym=abs(Y2)/(N/2);
Ym=Ym(1:N).^2;
plot(f,Ym);
title('Espectro Posterior a Filtragem (F. Médias)');
ylabel('Energia');
xlabel('Frequência (Hz)');
grid;
axis([300 2e+3 0 1e-4]);
% Filtragem das F. Altas
Y3=X.*H4';
subplot(2,2,4);
Ya=abs(Y3)/(N/2);
Ya=Ya(1:N).^2;
plot(f,Ya);
title('Espectro Posterior a Filtragem (F. Altas)');
ylabel('Energia');
xlabel('Frequência (Hz)');
grid;
axis([300 2e+3 0 1e-4]);
%% Anti Transformada de Fourier
%Áudio Original
figure
subplot(2,2,1);
plot(t,Samp(:,1));
title('Áudio Original');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
ylabel('Amplitude');
grid;
%Áudio F. Baixas
y1=real(ifft(Y1));
subplot(2,2,2);
plot(t,y1);
ylabel('Amplitude');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
title('Audio Filtrado (F. Baixas)');
grid;
%Áudio F. Médias
y2=real(ifft(Y2));
subplot(2,2,3);
plot(t,y2);
ylabel('Amplitude');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
title('Audio Filtrado (F. Médias)');
grid;
y3=real(ifft(Y3));
%Áudio F. Altas
subplot(2,2,4);
plot(t,y3);
ylabel('Amplitude');
xlabel('Tempo (s)');
axis([0 T -.5 .5]);
title('Audio Filtrado (F. Altas)');
grid;