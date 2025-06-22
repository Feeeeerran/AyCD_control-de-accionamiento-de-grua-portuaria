clc; 


%% Controlador del trolley ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
num_t = [1];
den_t = [m_t b_t 0];
G_t = tf(num_t, den_t);  % Crear función de transferencia
p_t = pole(G_t)       % Calcular polo

%+ Mcx si es que el TLK esta en ON
m_l = M_s 
factor_alejamiento_t = 5;
p_controlador_t = factor_alejamiento_t*min(p_t)
n_t = 2.5;
Jt_eq_prima = m_t + (1/(r_td^2))*((i_t^2)*Jt_er + Jt_el) + m_l;
bt_eq_prima = (1/(r_td^2))*((i_t^2)*b_tm + b_td) + b_t;

Kd_t = ((n_t*p_controlador_t*r_td)/i_t)*Jt_eq_prima - (r_td/i_t)*bt_eq_prima
Kp_t = (n_t*(p_controlador_t^2)*r_td*Jt_eq_prima)/i_t
Ki_t = ((p_controlador_t^3)*r_td*Jt_eq_prima)/i_t


%% Controlador del hoist ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
num_h = [1];
den_h = [-Jh_eq/i_h -bh_eq/i_h 0];
G_h = tf(num_h, den_h);             % Crear función de transferencia
p_h = pole(G_h)                     % Calcular polo
Ganancias
factor_alejamiento_h = 5;
p_controlador_h = factor_alejamiento_h*min(p_h)
n_h = 2.5;
%Jh_eq_prima = m_t + (1/(r_td^2))*((i_t^2)*Jt_er + Jt_el) + m_l;
%bt_eq_prima = (1/(r_td^2))*((i_t^2)*b_tm + b_td) + b_t;

Kd_h = (bh_eq/i_h) - ((n_h*p_controlador_h)/i_h)*(Jh_eq + ((m_l*r_hd)/2))
Kp_h = - ((n_h*(p_controlador_h^2))/i_h)*(Jh_eq + ((m_l*r_hd)/2))
Ki_h = - ((p_controlador_h^3)/i_h)*(Jh_eq + ((m_l*r_hd)/2))