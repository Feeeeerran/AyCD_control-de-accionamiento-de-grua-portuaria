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
T_s0 = 20 / 100;    % [s] Periodo de muestreo
fs0 = 1 / T_s0;     % [Hz] Frecuencia de muestreo
H_SS = H_c;         % Altura de seguridad 


%% Condiciones iniciales
CI = [
    % x_t0
    -3.66       
    % dx_t0       
    0       
    % y_h0
    H_SS       
    % dy_h0
    0       
];

%% Limites operativos de movimiento

% Traslacion del carro
x_t_lim = [-30 50]; % [m] En x_t = 0 el carro se encuentra sobre el borde del muelle
dx_t_lim = 4;       % [m/s] Velocidad maxima cargado o sin carga
ddx_t_lim = 0.8;    % [m/s^2] Aceleracion maxima con y sin carga

% Izaje de carga
y_h_lim = [-20 40]; % [m] Altura de izaje (valores negativos representan dentro de barco
dy_h_lim = [1.5 3]; % [m/s] Velocidad maxima de izaje (1) Con carga (2) Sin carga
ddy_h_lim = 0.75;   % [m/s^2] Aceleracion maxima con y sin carga

% set_param_limite = {
%     x_t_lims,     y_h_lims;
%     dx_t_lims,    dy_h_lims;
%     ddx_t_lims,   ddy_h_lims
% };
% 
