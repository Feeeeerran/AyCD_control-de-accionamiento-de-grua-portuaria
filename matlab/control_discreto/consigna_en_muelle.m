function [carro_cons, izaje_cons, t_carro, t_izaje] = consigna_en_muelle(Ts, w_c, set_param_lim, n_muelle, altura_izaje, tipo, h_ss, v_t, v_h)

% Se generan las consignas para la carga/descarga en muelle
% El tipo de accion, carga o descarga depende del parametro tipo
%   0 - Carga
%   1 - Descarga

% Se considera que el izaje viene con una altura constante (altura_izaje) a
% una velocidad de carro v_act inicial.

% Se suponen 3 zonas de carga/descarga en muelle, distanciadas por la misma
% distancia del propio container entre si. Se enumeran de forma incremental
% a medida que la posicion se aleja del barco. El carro debe posicionarse
% sobre el container.
distancia = (n_muelle * 2 * w_c) - (w_c/2);


% Carga desde muelle hacia barco ====================
if tipo == 0
    % Movimiento del carro hacia barco
    [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts, -distancia, 0, 0, set_param_lim(1), set_param_lim(1), set_param_lim(2));
    % Movimiento de izaje hacia y_max
    [y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, h_ss, v_h, altura_izaje, 0, set_param_lim(3), set_param_lim(4));
end

% Descarga desde barco a muelle =================
if tipo == 1
    % Movimiento del carro hacia muelle
    [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts, 0, v_t, -distancia, 0, set_param_lim(1), set_param_lim(2));
    % Movimiento de izaje
    [y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, altura_izaje, 0, h_ss, set_param_lim(3)/2 ,set_param_lim(3), set_param_lim(4));
end

    % Encapsulamiento de las consignas
    carro_cons = [x_t; dx_t]';
    izaje_cons = [y_h; dy_h]';



end

