clear;
clc;

input = 0:0.5:10;
output = [0,0,0,0,0,0,0.00125,0.0025,0.00375,0.0048,0.0064,0.0083,0.0105,0.01255,0.0149,0.017,0.019,0.0215,0.02375,0.026,0.0285];

figure(1);
plot(input,output);
xlabel 'Voltage'
ylabel 'Force'
title 'Steady-State Response Plot'