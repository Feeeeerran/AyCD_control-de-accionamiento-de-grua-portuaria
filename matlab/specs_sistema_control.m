

% Valores de muestreo para la estructura y funciones del sistema de control
% y proteccion por niveles


% El sistema hibrido de control y proteccion debe incluir 3 niveles
% jerarquicos

%% Nivel 2 - Control regulatorio
Ts2 = 1 / 1000;     % [s] Periodo de muestreo
fs2 = 1 / Ts2;      % [Hz] Frecuencia de muestreo

%% Nivel 1 - Control supervisor
Ts1 = 20 / 1000;    % [s] Periodo de muestreo
fs1 = 1 / Ts1;      % [Hz] Frecuencia de muestreo

%% Nivel 0 - Seguridad / proteccion
Ts0 = 20 / 1000;    % [s] Periodo de muestreo
fs0 = 1 / Ts0;      % [Hz] Frecuencia de muestreo