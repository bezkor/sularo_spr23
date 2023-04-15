function dx = model_vor(t,x,flight_plan,disturbance)
% Модель польоту по лінії заданного шляху

  % Констатанти моделі
  Ka = 0.35; % Коефіціент закону керування по азимуту
  Kz = 0.25;   % Коефіціент закону керування по відхиленню
  waypoint_radius = 5; % Гранична відстань до контрольної точки
  waypoint_number = size(flight_plan,1); % кількість точок маршруту
  dx=zeros(3,1);

  % Поточний сегмент маршруту
  persistent waypoint;
  if isempty(waypoint),
    waypoint = 1;
  end;

  % При проходженні кінцевої точки зупинити моделювання
  if waypoint > waypoint_number,
    return;
  end;

  % Поточне положення
  x_current=x(1);
  y_current=x(2);
  a_current=x(3);

  % Проміжна точка маршруту
  x_start =flight_plan(waypoint,1);
  y_start =flight_plan(waypoint,2);
  x_target=flight_plan(waypoint,3);
  y_target=flight_plan(waypoint,4);

  %  Імітація роботи системи DME
  DME=sqrt((x_target-x_current)^2+(y_target-y_current)^2);


  % Перевірка досягнення ПТМ
  if DME < waypoint_radius,
    waypoint = waypoint+1;
    return;
  end

  % Імітація роботи системи VOR
  % Сигнал VOR що приймає літак
  VOR_aircraft=atan2d(y_target-y_current,x_target-x_current);
  % Курс VOR що заданий ЛЗШ
  VOR_line = atan2d(y_target-y_start,x_target-x_start);

  % Навігаційний сигнал системи VOR
  VOR=VOR_line-VOR_aircraft;

  dZ = -DME*sind(VOR);

  %disp([waypoint,DME, VOR,dZ]);
  % Реалізація керування
  u=Ka*(VOR_aircraft - a_current) + Kz*dZ;

  %u=1;
  dx=dubins_car(t,x,u,disturbance(t));

end

