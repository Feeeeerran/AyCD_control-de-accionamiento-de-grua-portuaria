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


%% Notas
% Habria que desacoplar y tener en cuenta la elasticidad del cable en ambos
% controladores (trolley y hoist) y evitar la suposicion donde la salida
% del tambor es aprox al movimiento de la carga o carro.