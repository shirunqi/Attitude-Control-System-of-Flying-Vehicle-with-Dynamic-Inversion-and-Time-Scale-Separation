clc
clear all
close all

%% Çó½âde = -ke
syms e t
e = dsolve('De = - 2 * e','e(0) = 1','t')

ezplot(e);

e1 = dsolve('De1 = -k * e1')