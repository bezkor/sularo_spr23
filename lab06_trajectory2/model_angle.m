function dx = model_angle(t,x,flight_plan,disturbance)
% Модель польоту по лінії заданного шляху

  % Констатанти моделі
  Ka = 0.3; % Коефіціент закону керування по азимуту
  Kz = 1.5;   % Коефіціент закону керування по відхиленню
  waypoint_radius = 5; % Гранична відстань до контрольної точки
  waypoint_number = size(flight_plan,1); % кількість точок маршруту
  dx=zeros(3,1);

  % Поточний сегмент маршруту
  persistent waypoint;
  if isempty(waypoint),
    waypoint = 1;
  end

  % При проходженні кінцевої точки зупинити моделювання
  if waypoint > waypoint_number,
    return;
  endif

  % Поточне положення
  x_current=x(1);
  y_current=x(2);
  a_current=x(3);
  % Проміжна точка маршруту
  x_start =flight_plan(waypoint,1);
  y_start =flight_plan(waypoint,2);
  x_target=flight_plan(waypoint,3);
  y_target=flight_plan(waypoint,4);

  % Перевірка досягнення ПТМ
  if ((x_target-x_current)^2+(y_target-y_current)^2) < waypoint_radius^2,
    waypoint = waypoint+1;
    return;
  end

  % Заданий азимут польоту
  a_target=atan2d(y_target-y_current,x_target-x_current);
  a_line = atan2d(y_target-y_start,x_target-x_start);

  % Реалізація керування
  u=Ka*(a_target - a_current) - Kz*(a_line - a_target);
  %u=1;
  dx=dubins_car(t,x,u,disturbance(t));

end
