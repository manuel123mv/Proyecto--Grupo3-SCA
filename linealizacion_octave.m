clc

%%electroiman
l=0.05; %longitud
r=0.007938/2;%radio del nucleo
N=1295;  %numero de vueltas
u0=4*pi*1e-7; %permeabilidad en el vacio
ur=1.21;   %permeabilidad relativa del acero
Ab=pi*r^2;   %area seccion transversal bobina
L=u0*ur*N^2*Ab/l      %inductancia del electroiman

k= 0.30442;%parametro de fuerza electromagnetica

R=10  %resistencia de alambre en la bobina
g=9.76%gravedad
m=0.03%masa

xo1=0.02 %punto de equilibrio 
xo3=xo1*sqrt(m*g/k) %punto de equilibrio

A=[0 ,1, 0;
   2*k/m*xo3^2/xo1^3 ,0, -2*k/m*xo3/xo1^2;
   0 ,0, -R/L]

lamda=eig(A)


