function dx=model_air(t,x,u)
% Спрощена модель повздовжнього каналу ЛА у формі Коші
% dx=model_air(t,x,u)
% t - час інтегрування
% x = [alpha,omega,theta] - вектор стану
% u - сигнал керування (відхилення РВ)

  alpha=x(1);   % Кут атаки
  omega=x(2);   % Кутова швидкість повороту
  theta=x(3);   % Кут тангажа
  
  dx=zeros(size(x));
  dx(1)=-0.313*alpha+56.7*omega+0.234*u;
  dx(2)=-0.0139*alpha-0.426*omega+0.0203*u;
  dx(3)=56.7*omega;

