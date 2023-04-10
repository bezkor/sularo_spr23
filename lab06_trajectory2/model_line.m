function dx = model_line(t,x,flight_plan,disturbance)
% Модель польоту по лінії заданного шляху з лінійною коррекцією по відхиленню

  % Констатанти моделі
  Ka = 0.35; % Коефіціент закону керування по азимуту
  Kz = 0.25;   % Коефіціент закону керування по відхиленню
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

  % Розрахунок функції лінії заданного шляху
  % f(x,y)= a_line*x + b_line*y + c_line
  a_line = y_target - y_start;
  b_line = -(x_target - x_start);
  % Нормалізація рівняння лінії
  n_line = norm([a_line, b_line]);
  a_line = a_line/n_line;
  b_line = b_line/n_line;
  % Обчислення коефіціенту зміщення
  c_line = -(a_line*x_target+b_line*y_target);

  % Розрахунок функції фінішної лінії (нормальної)
  % f(x,y)= a_finish*x + b_finish*y + c_finish
  a_finish = b_line;
  b_finish = -a_line;
  c_finish = -(a_finish*x_target+b_finish*y_target);

  % Бокове зміщення - відстань до ЛЗШ
  z_line = a_line*x_current + b_line*y_current + c_line;

  % Відстань до фінішної лінії
  l_line = a_finish*x_current + b_finish*y_current + c_finish;

  % Оцінка азимуту на ППМ
  a_target=atan2d(y_target-y_current,x_target-x_current);


  % Перевірка досягнення ПТМ
  if l_line < 0,
    waypoint = waypoint+1;
    return;
  end


  % Реалізація закону керування
  u=Ka*(a_target - a_current) + Kz*z_line;

  % Обрахунок реакції
  dx=dubins_car(t,x,u,disturbance(t));

end
