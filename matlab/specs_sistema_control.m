clc;

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


%% Limites operativos de movimiento

% Traslacion del carro
x_t_lims = [-30 50]; % [m] En x_t = 0 el carro se encuentra sobre el borde del muelle
dx_t_lims = 4;       % [m/s] Velocidad maxima cargado o sin carga
ddx_t_lims = 0.8;    % [m/s^2] Aceleracion maxima con y sin carga

% Izaje de carga
y_h_lims = [-20 40]; % [m] Altura de izaje (valores negativos representan dentro de barco
dy_h_lims = [1.5 3]; % [m/s] Velocidad maxima de izaje (1) Con carga (2) Sin carga
ddy_h_lims = 0.75;   % [m/s^2] Aceleracion maxima con y sin carga