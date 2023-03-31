function dx=dubins_car(t,x,u,disturbance)
% Модель машини Дубінса
% Вхідні параметри
% t - час
% x - вектор стану
% u - значення керування
% disturbance - зовнішні збурення
%
% Вектор стану
% x(1) - координата x
% x(2) - координата y
% x(3) - кут напряму руху в градусах

% Константи машини Дубінса
u_bound = 30;    % Обмеження на керування в град/с
V = 10;          % Швидкість переміщення  в м/с

% Обмеження керування
if (u>u_bound), 
    u = u_bound; 
end;
if (u<-u_bound),
    u = -u_bound; 
end;

% Рівняння математичної моделі
dx(1) = V*cosd(x(3)); % приріст координати x
dx(2) = V*sind(x(3)); % приріст координати y
dx(3) = u;            % приріст кута повороту

dx=dx(:)+disturbance(:);

end