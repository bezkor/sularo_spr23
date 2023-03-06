function dx=model_bound(t,x)
% Стабілізація кута тангажу літака обмеженим PID регулятором
% Змінні вектора х:
% x(1) - кут атаки
% x(2) - кутова швидкість відносно осі z
% x(3) - кут тангажа
% x(4) - значення інтегральної складової PID регулятора

   alpha=x(1);
   omega=x(2);
   theta=x(3);
   integr=x(4);

   % Заданий кут стабілізації
   theta_ref = 0;
   if t>1,
     theta_ref = 1;
   end;

   % Зовнішнє збурення у вигляді еквівалентного сигналу керування
   disturbance = 0;
   if t>30,
     disturbance = 0.25;
   end;

   % Налаштування PID
   KP=5;
   KD=30;
   KI=1;

   % Реалізація PID регулювання
   err=theta_ref-theta; %Помилка стабілізації
   u=KP*err-KD*omega+KI*integr; %

   % Обмеження вихідного сигналу PID регулятора
   if u>0.5,
     u = 0.5;
   end;
   if u<-0.5,
     u = -0.5;
   end;

   % Реакція літака
   dx=model_air(t,[alpha;omega;theta],u+disturbance);

   % Розширення моделі системи з урахуванням PID
   dx(4) = err;

