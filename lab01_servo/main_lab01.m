% Пошук оптимального значення PID регулятора
clear all
% Перевірка середовища виконання
if exist('OCTAVE_VERSION', 'builtin') ~= 0,
  disp('Running in Octave');
  pkg load control;
else
  disp('Running in Matlab');
end;

% ========== START EDIT ==================
% Значення налаштувань PID регулятора
KP=6
KI=1
KD=5
% ========== STOP EDIT ===================


% Характеристики двигуна постійного струму
J = 0.01; % Момент інерції
b = 0.1;  % Момент тертя
K = 0.01; % Електрична постійна
R = 1;    % Опір якоря
L = 0.5;  % Індуктивність якоря

% Передатна функція двигуна ПС
s=tf('s')
DC_motor = K/(s*((J*s+b)*(L*s+R)+K^2))

% Передатна функція PID регулятора
TF_pid=@(Kpid) pid(Kpid(1),Kpid(2),Kpid(3));

% Передатна функція сервоприводу
TF_servo=@(Kpid) feedback(DC_motor*TF_pid(Kpid),1);
% Передатна функція помилки сервоприводу
TF_error=@(Kpid) feedback(1,DC_motor*TF_pid(Kpid));
% Передатна функція помилки сервоприводу по збуренню
TF_disturbance=@(Kpid) feedback(DC_motor,TF_pid(Kpid));

% Передатна функція сервоприводу з ручними налаштуваннями
WservoM=TF_servo([KP,KI,KD])
% Передатна функція реакції на зовнішнє збурення
WdistM=TF_disturbance([KP,KI,KD])

% Порівняння перехідних процесів
figure(1)
step(WservoM,WdistM, 15);


