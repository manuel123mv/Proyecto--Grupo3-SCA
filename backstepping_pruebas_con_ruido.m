function backstepping_pruebas()
to=0; % start time
tf=5; % end time
t=linspace(to,tf,400);
xo(1)=0.05; % posicion inicial
xo(2)=0.0001; %velocidad inicial
xo(3)=0.0001; %corriente inicial
xo(4)=0;    %control u
xo(5)=0;    %ruido
[t,x] = ode45(@(t,x)nonlinear(t,x),t,xo);

%xd=0.08*ones(1,length(t));
f=1;
w=2*pi*f;
xd=0.04+0.02*sin(w*t);

figure(1)
plot(t,x(:,1),t,xd); grid minor
legend('x1','x deseado'); xlabel('Tiempo s'); ylabel('Posicion m');
title('Seguimiento de posicion'); 

figure(2)
plot(t,x(:,2)); grid minor
legend('v(t)'); xlabel('Tiempo s'); ylabel('Velocidad m/s');
title('Velocidad de movimiento');

figure(3)
plot(t,x(:,3)); grid minor
legend('i(t)'); xlabel('Tiempo s'); ylabel('Corriente A');
title('Corriente en la bobina');

u=[0 ; diff(x(:,4))/(t(2)-t(1))]; %%derivamos la solucion de u,
figure(4)
plot(t,u); grid minor
legend('u(t)'); xlabel('Tiempo s'); ylabel('Voltaje V');
title('Se�al de Control: Voltaje');

d0=[0 ; diff(x(:,5))/(t(2)-t(1))]; %%derivamos la solucion de d0,
figure(5)
plot(t,d0); grid minor
legend('d0(t)'); xlabel('Tiempo s'); ylabel('Amplitud');
title('Se�al de Ruido');



function [xdot] = nonlinear(t,x)
% nonlinear model to integrate with ODE45
% parameters
R=10;%resistencia
L= 0.001381;%inductancia de electroiman
g=9.78;%gravedad
m=0.030;%masa de la bola
k= 0.30442;%parametro de fuerza electromagnetica

%ganancias para el control
k1=10; k2=10; k3=10;

%tracking xdeseado
f=1;
w=2*pi*f;
%xd=0.02;

xd=0.04+0.02*sin(w*t);
xd1=0.02*cos(w*t)*w;
xd2=-0.02*sin(w*t)*w^2;
xd3=-0.02*cos(w*t)*w^3;

%a�adir Ruido 
d0=0.01*(-1+2*rand)+(t>2 && t<2.01)*0.1; %Ruido mas un impulso en 2 seg

x(1)=x(1)+d0;  if(x(1)<0) x(1)=0.0001; endif
x(2)=x(2)+d0;  
x(3)=x(3)+d0;  if(x(3)<0) x(3)=0.0001; endif
% backstepping control

x1_=x(1)-xd;
E1=x(2)+k1*x1_-xd1;
E2=g-k/m*x(3)^2/x(1)^2+(k1+k2)*E1+(1-k1^2)*x1_-xd2;
v=-E2*(k1+k2+k3)-E1*(-k1*k2-k2^2+2-k1^2)-x1_*(-2*k1-k2+k1^3)+xd3;
u=-x(1)^2*m*L/(2*k*x(3))*(v-2*k/m*x(3)^2/x(1)^2*(x(2)/x(1)+R/L));

xdot=[x(2);
      g-k/m*x(3)^2/x(1)^2;
      -R/L*x(3)+u/L; 
      u;
      d0];  

endfunction
endfunction
