clc;

%% Hoist - Controlador PID para el sistema de izaje

% Se desarrolla el modelo dinamico, descartando T_hb, T_hEb y T_hdl se
% obtiene T_hm = Jh_eq'(TLK) ddl_h + bh_eq' dl_h

% Siendo Jh_eq' = Jh_eq_aux + Jh_mc(TLK)
% En esta instancia se calcula un Kp que solo tendra en cuenta a Jh_eq_aux
% y a posteriori en Simulink se agrega Jh_mc(TLK) para que la ganancia
% varie en funcion de la masa de la carga.

% Donde
Jh_eq_aux = ((M_s * r_hd) / (2 * i_h)) - ((Jh_eq * 2 * i_h) / r_hd);
bh_eq_prima = (2 * bh_eq * i_h) / r_hd;


% Se genera una funcion de transferencia para conocer los polos del sistema
% tf(num, den)
G_h = tf(1 , [Jh_eq_aux bh_eq_prima 0]);

% Se encuentran los polos
polos_h = pole(G_h);

disp('Polos del modelo dinamico de sistema de izaje')
fprintf('   p1 = %f  \n   p2 = %f  \n',polos_h(1), polos_h(2));

% ==================  SINTONIA SERIE  =========================
% w_n = 10 * polos_h(2);
% n = 3;
% 
% Kp_h = n * w_n^2;
% Ki_h = w_n^3;
% Kd_h = n * w_n;


% ==================  ASIGNACION DE POLOS  ====================
p = 25 * polos_h(2);  % Polo en eje real alejado 20 veces del polo del sistema
zitta = 1.3;          % Factor de amortiguamiento
T_r = 1;            % Settling time
w_n = 3/ T_r;        % Frecuencia natural calculada por tabla

Kp_h = (2 * zitta * w_n * p + w_n^2);
Ki_h = (w_n^2 * p);
Kd_h = (2 * zitta * w_n + p);


disp('Ganancias del controlador de izaje')
fprintf('   Kp_h = %f\n   Ki_h = %f\n   Kd_h = %f\n\n\n',Kp_h, Ki_h, Kd_h);

%% Trolley - Controlador PID


% Se desarrolla el modelo dinamico, descartando T_tb se
% obtiene T_tm = Jt_eq' ddx_t + bt_eq' dx_t

% Donde
Jt_eq_prima = (i_t / r_td) * Jt_eq + (r_td / i_t) * m_t;
bt_eq_prima = (i_t / r_td) * bt_eq + (r_td / i_t) * b_t;

% Se genera una funcion de transferencia para conocer los polos del sistema
% tf(num, den)
G_t = tf(1 , [Jt_eq_prima bt_eq_prima 0]);

% Se encuentran los polos
polos_t = pole(G_t);

disp('Polos del modelo dinamico de sistema de carro')
fprintf('   p1 = %f  \n   p2 = %f  \n',polos_t(1), polos_t(2));

% ==================  SINTONIA SERIE  =========================
% w_n = 100 * polos_t(2);
% n = 3;
% 
% Kp_t = (n * w_n^2) * Jt_eq_prima;
% Ki_t = (w_n^3) * Jt_eq_prima;
% Kd_t = (n * w_n) * Jt_eq_prima - bt_eq_prima;

% ==================  ASIGNACION DE POLOS  ====================
p = 0.01 * polos_t(2);  % Polo en eje real alejado 20 veces del polo del sistema
zitta = 1.3;          % Factor de amortiguamiento
T_r = 1;            % Settling time
w_n = 1 / T_r;        % Frecuencia natural calculada por tabla

Kp_t = (2 * zitta * w_n * p + w_n^2) * Jt_eq_prima;
Ki_t = (w_n^2 * p) * Jt_eq_prima;
Kd_t = (2 * zitta * w_n + p) * Jt_eq_prima - bt_eq_prima;

% ================== TUNE PID =================================
% Kp_t = 814.248336359987;
% Ki_t = 62.1556013121662;
% Kd_t = 2666.69590556106;

disp('Ganancias del controlador de carro')
fprintf('   Kp_t = %f\n   Ki_t = %f\n   Kd_t = %f\n',Kp_t, Ki_t, Kd_t);


%% Notas
% Habria que desacoplar y tener en cuenta la elasticidad del cable en ambos
% controladores (trolley y hoist) y evitar la suposicion donde la salida
% del tambor es aprox al movimiento de la carga o carro.




