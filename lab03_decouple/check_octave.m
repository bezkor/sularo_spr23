% Перевірка середовища виконання
if exist('OCTAVE_VERSION', 'builtin') ~= 0,
  disp('Running in Octave');
  pkg load control;
else
  disp('Running in Matlab');
end;
