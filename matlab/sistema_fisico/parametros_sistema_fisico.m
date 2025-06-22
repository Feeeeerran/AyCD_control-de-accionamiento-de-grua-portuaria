clc;


%% Parámetros generales del sistema:
Y_t0 = 45;              % [m] Altura fija de poleas de suspensión de izaje en el carro desde nivel de muelle.
H_c = 2.59;             % [m] Altura de container según estándar ISO.
W_c = 2.44;             % [m] Ancho de container según estándar ISO.
M_s = 15000;            % [kg] Masa del spreader + headblock (sin container).
M_cmax = 50000;         % [kg] Masa máxima de container totalmente cargado.
M_cmin = 2000;          % [kg] Masa mínima de container vacío.
g = 9.80665;            % [m/s^2] Aceleración gravitatoria.


%% Parámetros de contacto (carga apoyada):
K_cy = 1.8e9;           % [N/m] Rigidez por compresión vertical (carga comprime apoyo).
b_cy = 1.0e7;           % [N/(m/s)] Amortiguamiento por compresión vertical.
b_cx = 1.0e6;           % [N/(m/s)] Fricción de arrastre horizontal por contacto vertical.

%% Cable de acero de izaje (parámetros unitarios):
K_hwu = 236e6;          % [N/m]/m Rigidez unitaria a tracción del cable.
b_hwu = 150;            % [N/(m·s)]/m Amortiguamiento unitario a tracción del cable.
L_h0 = 110;             % [m] Longitud fija del cable desplegado.

%% Accionamiento del sistema de izaje:
r_hd = 0.75;            % [m] Radio primitivo del tambor (una corrida de cable).
J_hd = 3800;            % [kg·m^2] Momento de inercia del eje lento (tambor + freno emergencia + salida caja).
b_hd = 8.0;             % [N·m/(rad/s)] Fricción viscosa eje lento.
b_hEb = 2.2e9;          % [N·m/(rad/s)] Fricción viscosa freno de emergencia.
T_hEb_max = 1.1e6;      % [N·m] Torque máximo de freno de emergencia cerrado.
i_h = 22;               % Relación de transmisión de caja reductora.
J_hm = 30;              % [kg·m^2] Momento de inercia del eje rápido (motor + freno op. + entrada caja).
b_hm = 18.0;            % [N·m/(rad/s)] Fricción viscosa eje rápido.
b_hb = 100e6;           % [N·m/(rad/s)] Fricción viscosa freno operación cerrado.
T_hb_max = 50e3;        % [N·m] Torque máximo de freno operación cerrado.
tau_hm = 0.001;         % [s] Constante de tiempo del modulador de torque.
T_hm_max = 20e3;        % [N·m] Torque máximo motor izaje.

%% Carro y cable de acero del carro:
m_t = 30000;            % [kg] Masa equivalente del carro.
b_t = 90.0;             % [N/(m/s)] Fricción viscosa del carro.
K_tw = 480e3;           % [N/m] Rigidez a tracción del cable tensado del carro.
b_tw = 3.0e3;           % [N/(m/s)] Amortiguamiento del cable tensado del carro.

%% Accionamiento del carro:
r_td = 0.5;             % [m] Radio primitivo del tambor del carro.
J_td = 1200;            % [kg·m^2] Inercia del eje lento (tambor + salida caja).
b_td = 1.8;             % [N·m/(rad/s)] Fricción viscosa eje lento.
i_t = 30;               % Relación de transmisión de caja reductora.
J_tm = 7.0;             % [kg·m^2] Inercia del eje rápido (motor + freno + entrada caja).
b_tm = 6.0;             % [N·m/(rad/s)] Fricción viscosa eje rápido.
b_tb = 5e6;             % [N·m/(rad/s)] Fricción viscosa freno cerrado.
T_tb_max = 5e3;         % [N·m] Torque máximo freno carro cerrado.
tau_tm = 0.001;         % [s] Constante de tiempo del modulador de torque del carro.
T_tm_max = 4e3;         % [N·m] Torque máximo motor del carro.


%% Equivalencias
% Equivalencias resultantes de los despejes del modelo dinamico

% Del trolley
Jt_eq = J_tm + (J_td / i_t^2);          % [kg·m^2] Momento de inercia equivalente del carro
bt_eq = b_tm + (b_td/ i_t^2);           % [N·m/(rad/s)] Friccion viscosa equivalente del carro

% Del hoist
Jh_eq = J_hm + (J_hd / i_h^2);          % [kg·m^2] Momento de inercia equivalente del izaje
bh_eq = b_hm + (b_hd / i_h^2);          % [N·m/(rad/s)] Friccion viscosa equivalente del izaje



