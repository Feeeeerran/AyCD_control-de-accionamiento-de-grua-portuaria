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




