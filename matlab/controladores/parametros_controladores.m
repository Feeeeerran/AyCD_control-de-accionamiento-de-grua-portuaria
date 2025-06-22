clc;

%% Auxiliares
% Se contempla la F_hw dentro del desarrollo del controlador, implicando
% una variable m_l que varia segun el tipo de container. Se toma para este
% valor la media entre M_cmax y M_cmin sumado al M_s (spreader)
m_aux = M_s + (M_cmax + M_cmin) / 2;

%% Hoist - Controlador PID

% Se desarrolla el modelo dinamico, descartando T_hb, T_hEb y T_hdl se
% obtiene T_hm = Jh_eq' ddl_h + bh_eq' dl_h

% Donde
Jh_eq_prima = ((Jh_eq * 2 * i_h) / r_hd);
bh_eq_prima = (bh_eq * 2 * i_h) / r_hd;


% Se genera una funcion de transferencia para conocer los polos del sistema
% tf(num, den)
G_h = tf(1 , [Jh_eq_prima bh_eq_prima 0]);

% Se encuentran los polos
polos_h = pole(G_h);

disp('Polos del modelo dinamico de sistema de izaje')
fprintf('   p1 = %f  \n   p2 = %f  \n',polos_h(1), polos_h(2));

% Usando el metodo de pole placement se especifican los valores del
% polinomio caracteristico generico
% Siendo (s + p)(s^2 + 2 zitta w_n s + w_n^2)
p = 100;
zitta = 0.8;        % Factor de amortiguamiento
T_r = 0.5;          % Rise time
w_n = 2.5 / T_r;    % Frecuencia natural calculada por tabla


% Finalmente se satisfacen las igualdades desarrolladas para calcular las
% ganancias del controlador

Kp_h = (2 * zitta * w_n * p + w_n^2) * Jh_eq_prima;
Ki_h = (w_n^2 * p) * Jh_eq_prima;
Kd_h = (2 * zitta * w_n + p) * Jh_eq_prima - bh_eq_prima;

disp('Ganancias del controlador de izaje')
fprintf('   Kp_h = %f\n   Ki_h = %f\n   Kd_h = %f\n\n\n',Kp_h, Ki_h, Kd_h);

%% Trolley - Controlador PID


% Se desarrolla el modelo dinamico, descartando T_tb se
% obtiene T_tm = Jt_eq' ddx_t + bt_eq' dx_t

% Donde
Jt_eq_prima = (r_td/i_t) * m_t + (i_t/r_td) * J_tm + (1/(i_t * r_td)) * J_td;
bt_eq_prima = (r_td/i_t) * b_t + (i_t/r_td) * b_tm + (1/(i_t * r_td)) * b_td;

% Pruebas con otros valores
% Jt_eq_prima =  Jt_eq * i_t/r_td + m_t * (r_td / i_t);
% bt_eq_prima = bt_eq * i_t/r_td + b_t * (r_td / i_t);


% Se genera una funcion de transferencia para conocer los polos del sistema
% tf(num, den)
G_t = tf(1 , [Jt_eq_prima bt_eq_prima 0]);

% Se encuentran los polos
polos_t = pole(G_t);

disp('Polos del modelo dinamico de sistema de carro')
fprintf('   p1 = %f  \n   p2 = %f  \n',polos_t(1), polos_t(2));

% Usando el metodo de pole placement se especifican los valores del
% polinomio caracteristico generico
% Siendo (s + p)(s^2 + 2 zitta w_n s + w_n^2)
p = 0.3;
zitta = 1;      % Factor de amortiguamiento
T_r = 0.8;          % settling time
w_n = 2 / T_r;    % Frecuencia natural calculada por tabla


% Finalmente se satisfacen las igualdades desarrolladas para calcular las
% ganancias del controlador

Kp_t = (2 * zitta * w_n * p + w_n^2) * Jt_eq_prima;
Ki_t = (w_n^2 * p) * Jt_eq_prima;
Kd_t = (2 * zitta * w_n + p) * Jt_eq_prima - bt_eq_prima;

disp('Ganancias del controlador de carro')
fprintf('   Kp_t = %f\n   Ki_t = %f\n   Kd_t = %f\n',Kp_t, Ki_t, Kd_t);


%% Notas
% El controlador del trolley esta raro, creo que al suponer x_t = x_td y
% saltear la elasticidad del cable el controlador pierde efectividad. Si se
% prueba particularmente con los otros modelos dinamicos funciona super
% bien, el tema es cuando el F_tw se interpone.

% Probablemente haya que desacoplar esa fuerza de alguna forma.


