function [carro_cons, izaje_cons, t_carro, t_izaje] = consigna_en_muelle(Ts, w_c, set_param_lim, n_muelle, altura_izaje, h_ss, v_t, v_h, tipo)

% Se generan las consignas para la carga/descarga en muelle
% El tipo de accion, carga o descarga depende del parametro tipo
%   0 - Descarga desde barco
%   1 - Carga hacia barco

% Se considera que el izaje viene con una altura constante (altura_izaje) a
% una velocidad de carro v_act inicial.

% Se suponen 3 zonas de carga/descarga en muelle, distanciadas por la misma
% distancia del propio container entre si. Se enumeran de forma incremental
% a medida que la posicion se aleja del barco. El carro debe posicionarse
% sobre el container.
distancia = (n_muelle * 2 * w_c) - (w_c/2);


% perfil_velocidad_trapezoidal(Ts, x_0, v_0, x_f, v_f, v_max, a_max)
% Descarga desde barco a muelle =================
if tipo == 0
    % Movimiento del carro hacia muelle
    [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts, 0, v_t, -distancia, 0, set_param_lim(1), set_param_lim(2));
    % Movimiento de izaje
    [y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, altura_izaje, 0, 2* h_ss, -set_param_lim(3)/3 ,set_param_lim(3), set_param_lim(4));
    
    % Sincronizacion
    if numel(x_t) > numel(y_h)
        n_dif = numel(x_t) - numel(y_h);
        y_h = [y_h (ones(1,n_dif) * h_ss)];
        dy_h = [dy_h zeros(1, n_dif)];
        t_izaje = t_izaje + n_dif * Ts
    elseif numel(y_h) > numel(x_t)
        n_dif = numel(y_h) - numel(x_t);
        x_t = [x_t (ones(1, n_dif) * -distancia)];
        dx_t = [dx_t zeros(1, n_dif)];
        t_carro = t_carro + n_dif * Ts;
    else
    end
end

% perfil_velocidad_trapezoidal(Ts, x_0, v_0, x_f, v_f, v_max, a_max)
% Carga desde muelle hacia barco ====================
if tipo == 1
    % Movimiento del carro hacia barco
    [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts, -distancia, 0, 0, set_param_lim(1), set_param_lim(1), set_param_lim(2));
    % Movimiento de izaje hacia y_max
    [y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, h_ss, v_h, altura_izaje, 0, set_param_lim(3), set_param_lim(4));

    % Correccion del perfil (no termina en 0 para el carro
    p_fin = numel(x_t);

    for idx = p_fin : -1 : 1
        if x_t(idx) < 0 && idx ~= p_fin
            x_t = x_t(1: idx + 1);
            dx_t = dx_t(1: idx + 1);
            t_carro = numel(x_t) * Ts;
            break;
        end
    end

    % Sincronizacion
    if numel(x_t) > numel(y_h)
        n_dif = numel(x_t) - numel(y_h);
        y_h = [(ones(1, n_dif) * h_ss) y_h];
        dy_h = [zeros(1, n_dif) dy_h];
        t_izaje = t_izaje + n_dif * Ts;
    elseif numel(y_h) > numel(x_t)
        n_dif = numel(y_h) - numel(x_t);
        x_t = [(ones(1, n_dif) * -distancia) x_t];
        dx_t = [zeros(1, n_dif) dx_t];
        t_carro = t_carro + n_dif * Ts;
    else
    end
    
end
    % Encapsulamiento de las consignas
    carro_cons = [x_t; dx_t]';
    izaje_cons = [y_h; dy_h]';



end

