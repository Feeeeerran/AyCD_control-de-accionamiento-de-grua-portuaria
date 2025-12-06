function [carro_cons, izaje_cons] = operacion_barco_a_muelle(Ts, perfil_obstaculos, w_c, TLK, param_lim, h_ss, x, y, n_muelle, p_barco)
% Funcion para generar consignas de posicion y velocidad en la operacion a
% realizar desde barco hasta muelle

% p_barco representa la posicion en el arreglo del perfil de obstaculos
% El perfil de obstaculos ya viene con las alturas maximas calculadas

% Si los TLK estan activados es que se lleva una carga,
% por ende h_ss = h_ss * 2

if TLK
    h_ss = h_ss * 2;
end

% Se filtra perfil de obstaculos solo para la distancia a recorrer
% Se da vuelta porque se va desde barco a muelle
obs = flip(perfil_obstaculos(1:p_barco));

% Se calcula la distancia desde x = 0 hasta n_muelle
x_muelle = -((n_muelle * 2 * w_c) - (w_c / 2));

% Se calcula la altura maxima dentro del recorrido hasta llegar a x = 0


% Se lleva izaje hasta altura maxima y_max
[y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, y_ini, v_h, y_max, 0, set_param_lim(3), set_param_lim(4));




end

