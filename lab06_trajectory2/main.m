% Дослідження траекторії руху ЛА (2)

% Очистка змінних середовища
clear all
ode_opts = odeset('MaxStep',1e0);
tflight=[0:1:800];

% Проміжні точки маршруту
flight_plan_x=[0 250 750 1250 1500]
flight_plan_y=[0 0   500 0    0]


% Стартова позиція
start_pos=[flight_plan_x(1),flight_plan_y(1),0];

% Формування польотного плану
flight_plan=zeros(1,4);
for i=1:length(flight_plan_x)-1,
  flight_plan(i,:)=[flight_plan_x(i),...
                    flight_plan_y(i),...
                    flight_plan_x(i+1),...
                    flight_plan_y(i+1)];
end;

% Модель вітрових збурень
% (1) - складова збурення вздовж осі x
% (2) - складова збурення вздовж осі y
% (3) - моментра складова збурення = 0
disturbance = @(t) [0,0,0]';

% Польот за ЛЗШ кутовим методом коррекції
clear model_vor;
sim_model_vor=@(t,x) model_vor(t,x,flight_plan,disturbance);
[t,y_vor]=ode45(sim_model_vor,tflight,start_pos,ode_opts);

% Польот за ЛЗШ шляховим методом коррекції
clear model_line;
sim_model_line=@(t,x) model_line(t,x,flight_plan,disturbance);
[t,y_line]=ode45(sim_model_line,tflight,start_pos,ode_opts);

% Виконати коррекцію польотного плану:
% - розрахувати мінімальний радіус повороту
% - розрахувати упередження виконання маневру
% - провести симуляцію за скорегованим планом польоту
% - порівняти результати симуляції

% Корекція польотного плану
flight_plan_corr = flight_plan;

% Польот за ЛЗШ кутовим методом коррекції з корекцією плану
clear model_vor;
sim_model_vor_corr=@(t,x) model_vor(t,x,flight_plan_corr,disturbance);
[t,y_vor_corr]=ode45(sim_model_vor_corr,tflight,start_pos,ode_opts);

% Польот за ЛЗШ шляховим методом коррекції з корекцією плану
clear model_line;
sim_model_line_corr=@(t,x) model_line(t,x,flight_plan_corr,disturbance);
[t,y_line_corr]=ode45(sim_model_line_corr,tflight,start_pos,ode_opts);

% Візуалізація виконання польотного плану
figure(1)
plot(y_vor(:,1),y_vor(:,2),'g-','LineWidth',2);
hold on;
plot(y_line(:,1),y_line(:,2),'r-','LineWidth',3);
plot(flight_plan(:,[1,3])',flight_plan(:,[2,4])','b*--','LineWidth',1);
hold off;
axis equal
grid on
legend('Польот за ЛЗШ (VOR/DME)','Польот за ЛЗШ (GPS)','План польоту');
title('Порівняння траекторій польоту за кутовим та шляховим методом');


figure(2)
plot(y_vor_corr(:,1),y_vor_corr(:,2),'r-','LineWidth',2);
hold on;
plot(y_line_corr(:,1),y_line_corr(:,2),'g-','LineWidth',3);
plot(flight_plan_corr(:,[1,3])',flight_plan_corr(:,[2,4])','b*--','LineWidth',1);
hold off;
axis equal
grid on
legend('Польот за ЛЗШ скор (VOR)','Польот за ЛЗШ скор (GPS)','План польоту');
title('Порівняння скоригованих траекторій польоту за кутовим та шляховим методом');


figure(3)
plot(y_vor(:,1),y_vor(:,2),'r-','LineWidth',2);
hold on;
plot(y_vor_corr(:,1),y_vor_corr(:,2),'g-','LineWidth',3);
plot(flight_plan_corr(:,[1,3])',flight_plan_corr(:,[2,4])','b*--','LineWidth',1);
hold off;
axis equal
grid on
legend('Польот за ЛЗШ (VOR/DME)','Польот за ЛЗШ корр (VOR/DME)','План польоту');
title('Порівняння траекторій польоту за VOR/DME');

figure(4)
plot(y_line(:,1),y_line(:,2),'r-','LineWidth',2);
hold on;
plot(y_line_corr(:,1),y_line_corr(:,2),'g-','LineWidth',3);
plot(flight_plan_corr(:,[1,3])',flight_plan_corr(:,[2,4])','b*--','LineWidth',1);
hold off;
axis equal
grid on
legend('Польот за ЛЗШ (GPS)','Польот за ЛЗШ корр (GPS)','План польоту');
title('Порівняння траекторій польоту за GPS');

