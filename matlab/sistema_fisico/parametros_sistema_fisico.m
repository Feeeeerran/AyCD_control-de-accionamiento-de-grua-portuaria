clc; clear all;

%% Parámetros del sistema físico (grua y limites)

Y_t0 = 45;              % [m] Altura (fija) de poleas de suspensión de Izaje en el Carro
H_c = 2.59;             % [m] Altura de container stadard
W_c = 2.44;             % [m] Ancho de container standard
M_s = 15000;            % [kg] Masa de Spreader + Headblock (sin container)
M_cmax = 50000;         % [kg] Masa de Container/s a izar (máxima totalmente cargado)
M_cmin = 2000;          % [kg] Masa de Container Estándar vacío (mínima, sin carga interna).
g = 9.80665;            % [m/s2] Aceleración gravitatoria.

%% Reacción de vínculo del container

K_cy = 1.8e9;       % [N/m] Rigidez de compresión por contacto vertical (carga comprime apoyo).
b_cy = 10e6;        % [N/(m/s)] Fricción interna o amortiguamiento de compresión por contacto vertical.
b_cx = 1e6;         % [N/(m/s)] Fricción de arrastre horizontal por contacto vertical.

%% Cable de acero (wirerope) de izaje equivalente, parámetros unitarios (por metro de cable desplegado): 

K_hwu = 236e6;       % [(N/m)m] Rigidez unitaria a tracción (tensión por peso de carga). 
b_hwu = 150;         % [(N/(m/s))/m] Fricción interna o amortiguamiento unitario a tracción (rozamiento interno). 
L_h0 = 110;         % [m] Longitud de despliegue fijo de wirerope de izaje (desde el tambor hasta el extremo fijo), en adición a las 2 partes colgantes (péndulo) variable 𝑙ℎ(𝑡).


%% Accionamiento del Sistema de hoist: 

r_hd = 0.75;        % [m] Radio primitivo de tambor (enrollado helicoidal, 1 sola corrida de cable). 
Jh_el = 3800;       % [kg.m2] Momento de inercia equivalente del eje lento (tambor, disco de freno de emergencia y etapa de salida de caja reductora).
b_hd = 8;           % [(N.m)/(rad/s)] Coeficiente de Fricción mecánica viscosa equivalente del eje lento. 
b_hEb = 2.2e9;      % [(N*m)/(rad/s)] Coeficiente de Fricción viscosa equivalente del Freno de emergencia cerrado. 
T_hEb_max = 1.1e6;  % [N.m] Torque máximo de frenado del Freno de emergencia cerrado. 
i_h = 22;           % Relación de transmisión total de caja reductora de engranajes. 
Jh_er = 30;         % [kg.m2] Momento de inercia equivalente del eje rápido (motor, disco de freno de operación y etapa de entrada de caja reductora). 
b_hm = 18;          % [(N*m)/(rad/s)] Coeficiente de Fricción mecánica viscosa equivalente del eje rápido.
b_hb = 100e6;       % [(N*m)/(rad/s)] Coeficiente de Fricción viscosa equivalente del Freno de operación cerrado.
T_hb_max = 50e3;    % [N.m] Torque máximo de frenado del Freno de operación cerrado. 
tau_hm = 0.001;     % [s] Constante de tiempo de Modulador de Torque en motor-drive de izaje. 
T_hm_max = 20e3;    % [N.m] Torque máximo de motorización/frenado regenerativo del motor.


%% Trolley y cable de acero (wirerope) de carro equivalente: 

m_t = 30000;        % [kg] Masa equivalente de Carro, ruedas, efecto de carros catenaria, etc.
b_t = 90;           % [N/(m/s)] Coeficiente de Fricción mecánica viscosa equivalente del Carro. 
k_tw = 480e3;       % [N/m] Rigidez equivalente total a tracción de cable tensado de carro.
b_tw = 3e3;         % [N/(m/s)] Fricción interna o amortiguamiento total de cable tensado de carro. 


%% Accionamiento de traslación del trolley: 

r_td = 0.5;         % [m] Radio primitivo de tambor (enrollado helicoidal, 1 sola corrida de cable). 
Jt_el = 1200;       % [kg.m2] Momento de inercia equivalente del eje lento (tambor y etapa de salida de caja reductora). 
b_td = 1.8;         % [(N.m)/(rad/s)] Coeficiente de Fricción mecánica viscosa equivalente del eje lento. 
i_t = 30;           % Relación de transmisión total de caja reductora de engranajes. 
Jt_er = 7;          % [kg.m2] Momento de inercia equivalente del eje rápido (motor, disco de freno de operación y etapa de entrada de caja reductora). 
b_tm = 6;           % [(N.m)/(rad/s)] Coeficiente de Fricción mecánica viscosa equivalente del eje rápido. 
b_tb = 5e6;         % [(N.m)/(rad/s)] Coeficiente de Fricción viscosa equivalente del Freno de operación cerrado. 
T_tb_max = 5e3;     % [N.m] Torque máximo de frenado del Freno de operación cerrado. 
tau_tm = 0.001;     % [s] Constante de tiempo de Modulador de Torque en motor-drive de carro. 
T_tm_max = 4e3;     % [N.m] Torque máximo de motorización/frenado regenerativo del motor.

%% Equivalentes

% Del trolley
Jt_eq = Jt_er + (Jt_el / i_t^2);
bt_eq = b_tm + (b_td/ i_t^2);

% Del hoist
Jh_eq = Jh_er + (Jh_el / i_h^2);
bh_eq = b_hm + (b_hd / i_h^2);



