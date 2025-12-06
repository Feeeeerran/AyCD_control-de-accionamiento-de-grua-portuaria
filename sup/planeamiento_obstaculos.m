function [izaje_cons, carro_cons] = planeamiento_obstaculos(T_s, perfil_obstaculos, p_ini, p_fin, w_fila, set_param_lim, h_ss)

    % Posiciones inicial y final
    % x_pos_ini = (p_ini * w_fila) - w_fila / 2;
    % x_pos_fin = (p_fin * w_fila) - w_fila / 2;

    y_pos_ini = perfil_obstaculos(p_ini) + h_ss;

    cant_tramos = abs(p_fin - p_ini);

    if p_ini < p_fin
        obs = perfil_obstaculos(p_ini:p_fin);
    else
        obs = flip(perfil_obstaculos(p_ini:p_fin));
    end

    % ----------- IZAJE ---------------------------------------------------
    % Inicializacion de parametros
    posiciones_izaje = [];
    velocidades_izaje = [];
    tiempos_izaje = zeros(1, cant_tramos);
    elm_izaje = zeros(1, cant_tramos);

    % Se obtienen valores inciales del perfil de obstaculos
    y_act = y_pos_ini;
    y_max = max(obs);

    for tramo = 1:cant_tramos
        if tramo == 1
            % Primer tramo siempre se hace subida a y_max
            [y_rel, dy_rel, t_tramo] = perfil_velocidad_trapezoidal(T_s, y_max, set_param_lim(3), set_param_lim(4));
            
            % Se generan consignas de posicion e izaje
            posiciones_izaje  = [posiciones_izaje (y_rel + y_pos_ini)];
            velocidades_izaje = [velocidades_izaje dy_rel];

            tiempos_izaje(1) = t_tramo;
            elm_izaje(1) = numel(y_rel);

            % Se marca cual es la ultima posicion consigna
            y_act = posiciones_izaje(end);
            continue
        end

        % Tramos siguientes
        % Se actualiza el y_max en funcion de lo que queda del perfil de
        % obstaculos

        sig_y_max = max(obs(tramo:end));

        if sig_y_max >= y_max
            tiempos_izaje(tramo) = 0;
            elm_izaje(tramo) = 1;
            posiciones_izaje = [posiciones_izaje y_act];
            velocidades_izaje = [velocidades_izaje 0];
            continue
        end

        % Nuevo y_max (empieza a descender)
        y_prev = y_act;
        y_max = sig_y_max + h_ss;

        % Se calcula un nuevo perfil trapezoidal de descenso
        [y_rel, dy_rel, t_tramo] = perfil_velocidad_trapezoidal(T_s, (y_prev - y_max), set_param_lim(3), set_param_lim(4));

        posiciones_izaje = [posiciones_izaje (-y_rel + y_prev)];
        velocidades_izaje = [velocidades_izaje -dy_rel];

        tiempos_izaje(tramo) = t_tramo;
        elm_izaje(tramo) = numel(y_rel);
        y_act = y_max;
    end



    % ---------------- CARRO -----------------------------------------------
    % Inicializacion de parametros
    posiciones_carro  = [];
    velocidades_carro = [];
    tiempos_carro = zeros(1, cant_tramos);


    % Altura del obstaculo adyacente
    y_ady = obs(1 + (p_ini < p_fin));

    % Encuentra el instante en el que se supera la altura del obstáculo adyacente
    for k = 1:elm_izaje(1)
        if posiciones_izaje(k) >= (y_ady + h_ss)
            posiciones_carro  = zeros(1, k);
            velocidades_carro = zeros(1, k);
            break
        end
    end

    % Una vez superado el obstaculo adyacente, se puede empezar a trasladar
    % el carro, entonces se genera el perfil de velocidad trapezoidal
    [x_rel, dx_rel, ~] = perfil_velocidad_trapezoidal(T_s, numel(obs) * w_fila, set_param_lim(1), set_param_lim(2));

    % Se asignan posiciones y velocidades consigna
    posiciones_carro  = [posiciones_carro x_rel];
    velocidades_carro = [velocidades_carro dx_rel];

    elm_carro = [numel(posiciones_carro)-numel(x_rel), numel(x_rel)];


    % Calculo de los tiempos y elementos por tramo
    n_tramo_aux = 1;
    t_ant = 0;

    for k = 1:numel(posiciones_carro)
        if posiciones_carro(k) >= (n_tramo_aux * w_fila)
            tiempos_carro(n_tramo_aux) = (k * T_s) - t_ant;
            elm_carro(n_tramo_aux) = k - (t_ant/T_s);
            t_ant = k * T_s;
            n_tramo_aux = n_tramo_aux + 1;
            if n_tramo_aux > cant_tramos
                % Corte para no ir hasta el final
                break
            end
        end
    end


    % -------- SINCRONIZACIÓN ----------------------------------------------
    % Se sincronizan los elementos y tiempos en las consignas de movimiento
    for tramo = 1:cant_tramos
        if tiempos_izaje(tramo) == 0
            t_c = tiempos_carro(tramo);
            pos_i = sum(elm_izaje(1:tramo-1));

            n = uint16(t_c / T_s);

            % Se actualiza la posicion de izaje fija cuando para y_max
            posiciones_izaje  = [posiciones_izaje(1:pos_i), repmat(posiciones_izaje(pos_i), 1, n), posiciones_izaje(pos_i+1:end)];
            % Se mete en 0 la consigna de velocidad de izaje
            velocidades_izaje = [velocidades_izaje(1:pos_i), zeros(1, n), velocidades_izaje(pos_i+1:end)];

            tiempos_izaje(tramo) = t_c;
            elm_izaje(tramo) = n;
        end
    end

    % Encapsulando los resultados en arreglos
    izaje_cons = [posiciones_izaje; velocidades_izaje]';
    carro_cons = [posiciones_carro; velocidades_carro]';


    % izaje_cons = dictionary(["posicion","velocidad","t","elementos"], {posiciones_izaje, velocidades_izaje, tiempos_izaje, elm_izaje});
    % carro_cons = dictionary(["posicion","velocidad","t","elementos"], {posiciones_carro, velocidades_carro, tiempos_carro, elm_carro});
end