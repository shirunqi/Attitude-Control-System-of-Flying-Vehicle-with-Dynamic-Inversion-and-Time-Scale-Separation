clc
clear all
close all

syms theta gamma
A(1,:) = [0 sin(theta) 1];
A(2,:) = [sin(gamma) cos(theta) * cos(gamma) 0];
A(3,:) = [cos(gamma) -cos(theta) * sin(gamma) 0];

B = inv(A)