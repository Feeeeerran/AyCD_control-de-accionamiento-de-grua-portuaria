

% Valores de muestreo para la estructura y funciones del sistema de control
% y proteccion por niveles


% El sistema hibrido de control y proteccion debe incluir 3 niveles
% jerarquicos

%% Nivel 2 - Control regulatorio
T_s2 = 1 / 1000;     % [s] Periodo de muestreo
fs2 = 1 / T_s2;      % [Hz] Frecuencia de muestreo

%% Nivel 1 - Control supervisor
T_s1 = 20 / 1000;    % [s] Periodo de muestreo
fs1 = 1 / T_s1;      % [Hz] Frecuencia de muestreo

%% Nivel 0 - Seguridad / proteccion
T_s0 = 20 / 1000;    % [s] Periodo de muestreo
fs0 = 1 / T_s0;      % [Hz] Frecuencia de muestreo