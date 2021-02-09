
clc, clear all, close all;
% open sound file
filename = 'one'
filename = strcat(filename,'.wav')
path = strcat('Speech/',filename);
[x,Fs] = audioread(path);
% Filling the header file with samples
format long
file = fopen('signal.h', 'wt' );
fprintf(file, '\r\n#define S_L  %d // raw signal length', length(x));
fprintf(file, '\r\n#define Fs   %d// sampling frequency\r\n\n\n', Fs);
fprintf(file, 'float sig[S_L] = {\r\n');
for i = 1:length(x)-1
    fprintf(file, '%f,', x(i));
end
fprintf(file, '%f\r\n}; // samples',x(end));
fclose(file);