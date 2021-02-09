clc, clear all, close all;
% open sound file
filename = 'one'
filename = strcat(filename,'.wav')
path = strcat('Speech/',filename);
info    = audioinfo(path);
[x,Fs] = audioread(path);
N_mel = 12;
x = x'; % transposer x (colonne en ligne)
% figure(1)
% plot(x)
% Preparation
N = 2048; % number of fft data points
W = N;   % window size
H = N/4;  % hope size, c'est l'avancement dans le signal
% compute the hamming window
T = 0:N-1;  % variable temporelle de la fenetre
h = 0.54 - 0.46*cos(2*pi*T/N);
%plot(T,h)
% verifier si notre signal est un multiple de la taille de la fenetre W
% sinon ajouter des zeros (zero padding)
r = rem(length(x), W)
if  r ~= 0
    x = [x, zeros(1, W-r)];
end
f = (0:1/N:1/2)*Fs; % variable frequentielle
N_frames = ((length(x) - N)/H) + 1; % total number of frames
% Creation d'une matrice pour sauvegarder les fft partielles (stft)
% lignes : nombres de parties
% colonnes : taille de la fenetre
S = zeros(N_frames, N/2+1);
sfft = zeros(N_frames, N);
Mel = gen_mel(Fs,N,N_mel);
MFCC = zeros(N_frames, N_mel);
% Computing fft
start = 1;
for i=1:N_frames
    % compute PSD
    y = x(start : start + N - 1);
%     figure(1)
%     plot(y)
    y = y .* h;
    sfft(i,:) = y;
    Y = fft(y);
    S(i,:) = abs(Y(1:N/2+1)/N); % here we have our PSD
    % multiply by mel filterbanks
    Sum = zeros(1, N_mel);
    for j=1:N_mel
        mul = S(i,:).*Mel(j,:);
%         figure(1)
%         subplot(3,1,1)
%         plot(S(i,:));
%         subplot(3,1,2)
%         plot(Mel(j,:));
%         subplot(3,1,3)
%         plot(mul);
        Sum(j) = log10(sum(mul));
    end
    MFCC(i,:) = dct(Sum);
    start = start + H;
    disp(start)
end

imagesc(flipud(MFCC'),'CDataMapping','scaled'), colorbar

%figure(2)