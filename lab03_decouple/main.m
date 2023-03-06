% Компенсація перехресних звязків
clear all

% Перевірка середовища виконання
check_octave;

% Формування матриці передатних функцій моделі
% Входи - відхилення руля напряму та елекронів
% Виходи - кут крену та швидкість розвороту
G = minreal(tf(dornier328()),1)

% Визначення матриці передатних функцій системи
% з повною компенсацією перехресних з'вязків
T = [ G(1,1), 0; 0, G(2,2)];

% Визначення матриці передатних функцій спрощеного перехресного регулятора
R=[1 -G(1,2)/G(1,1); -G(2,1)/G(2,2) 1];

% Визначення матриці передатних функції системи з спрощеним регулятором
GR=minreal(G*R,1);


% ============== user edit =============
% Визначення матриці передатних функцій
% перехресного регулятора з повною компенсацією
Ri=[1 0; 0 1];
% ======================================

% Визначення матриці передатних функції системи
% з повною компенсацією перехресних звязків
GRi=minreal(G*Ri,1);


% Візуалізація результатів
figure(1)
bode(1.1*T(1,1),G(1,1),GR(1,1),GRi(1,1))
legend('Ideal','Real','Simple','Full');

figure(2)
bode(1.1*T(1,2),G(1,2),GR(1,2),GRi(1,2))
legend('Ideal','Real','Simple','Full');

figure(3)
bode(1.1*T(2,1), G(2,1),GR(2,1),GRi(2,1))
legend('Ideal','Real','Simple','Full');

figure(4)
bode(1.1*T(2,2),G(2,2),GR(2,2),GRi(2,2))
legend('Ideal','Real','Simple','Full');


