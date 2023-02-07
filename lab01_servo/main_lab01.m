% Пошук оптимального значення PID регулятора
clear all
% Перевірка середовища виконання
if exist('OCTAVE_VERSION', 'builtin') ~= 0,
  disp('Running in Octave');
  pkg load control;
else
  disp('Running in Matlab');
end;

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

% Передатні функція сервоприводу
TF_servo=@(Kpid) feedback(DC_motor*TF_pid(Kpid),1);
TF_error=@(Kpid) feedback(1,DC_motor*TF_pid(Kpid));
TF_disturbance=@(Kpid) feedback(DC_motor,TF_pid(Kpid));

% Значення налаштувань PID регулятора
KP=6
KI=1
KD=5

% Функція оптимізації
qscore=@(Kpid) norm(TF_error(Kpid));

% Пошук оптимального рішення
options = optimset('TolX',1e-6,'Display','iter','TolFun',1e-6);
Opt_pid=fminsearch(qscore,[KP,KI,KD],options)

% Передатна функція сервоприводу з ручними налаштуваннями
WservoM=TF_servo([KP,KI,KD])
WdistM=TF_disturbance([KP,KI,KD])
% Передатна функція сервоприводу з оптимізованими налаштуваннями
Wservo=TF_servo(Opt_pid)
Wdist=TF_disturbance(Opt_pid)
% Порівняння перехідних процесів
figure(1)
step(WservoM,Wservo);
figure(2)
step(WdistM,Wdist);


% Порівняння показників якості
fprintf("Ручні налаштування :[ ");
fprintf("%g ",[KP,KI,KD]);
fprintf("]; ");
fprintf("Q = %f\n",qscore([KP,KI,KD]));

fprintf("Оптимізовані налаштування :[ ");
fprintf("%g ",Opt_pid);
fprintf("]; ");
fprintf("Q = %f\n",qscore(Opt_pid));
