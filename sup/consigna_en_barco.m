function [carro_cons, izaje_cons, t_carro, t_izaje] = consigna_en_barco(Ts, w_c, h_c, perfil_obs, set_param_lim, p_ini, p_fin, h_ss, v_t, v_h, tipo)

% Se generan las consignas para la carga/descarga en barco
% El tipo de accion, carga o descarga depende del parametro tipo
%   0 - Carga (desde y_max y x_t = 0)
%   1 - Descarga (desde y_c + H_SS y x_t = pos_barco)
%   2 - Movimiento dentro de barco (desde p_ini hasta p_fin)

% Algunas definiciones para stateflow
x_t = [0]; dx_t = [0];
y_h = [0]; dy_h = [0];



% *** Contemplar en carga, que pasa cuando en el recorrido cambia y_max?
% Habria que descender a esa nueva altura, pero superando la sill beam

% Carga en barco (hacia muelle) ======================
% Siempre se realiza izaje hasta y_max, luego en funcion del movimiento de
% izaje, se empieza el movimiento del carro cuando se supera la altura del
% obstaculo adyacente
if tipo == 0
    % Se filtra el perfil solo para el recorrido a realizar con altura basada
    % en distancia de seguridad. Tambien se calcula altura inicial de izaje
    obs = (perfil_obs(1:p_ini) * h_c);
    y_ini = perfil_obs(p_ini) + h_ss;

    % Se da vuelta el arreglo del perfil para el caso de carga desde barco
    % hacia muelle
    obs = flip(obs);
    
    % Calculo de altura maxima de izaje
    y_max = max(obs) + h_ss;

    % Se realiza izaje a y_max
    [y_h, dy_h, t_izaje] = perfil_velocidad_trapezoidal(Ts, y_ini, v_h, y_max, 0, set_param_lim(3), set_param_lim(4));
    
    % La consigna del carro se genera a partir del momento en que el izaje
    % supera la altura del obstaculo adyacente
    y_ady = obs(2)

    % La posicion del carro esta dada por la mitad de la posicion 
    % del obstaculo * ancho del container (que considera espaciado entre containers)
    x_t_ini = (p_ini * w_c) - (w_c/2);

    x_t_aux = [];
    dx_t_aux = [];
    
    for idx = 1:length(y_h)
        if y_h(idx) >= y_ady
            % Se genera perfil trapezoidal para el carro
            [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts,x_t_ini, 0, 0, set_param_lim(1), set_param_lim(1), set_param_lim(2));
            t_carro = t_carro + (length(x_t_aux) * Ts)
            x_t = [x_t_aux x_t];
            dx_t = [dx_t_aux dx_t];
            break;
        end
        x_t_aux(idx) = x_t_ini;
        dx_t_aux(idx) = 0;
    end
end


% Descarga en barco (desde muelle) ====================
% En este caso la logica es inversa a la carga, se piensa en el movimiento
% del carro y se ajusta el izaje en funcion de los osbtaculos que hay entre
% el muelle y la posicion final en barco
if tipo == 1

    % Se filtra el perfil de obstaculos para el recorrido a realizar
    obs = perfil_obs(1:p_ini) * h_c;
    x_t_fin = (p_ini * w_c) - (w_c / 2);
    y_h_fin = obs(end) + h_ss;

    % Consignas de carro
    [x_t, dx_t, t_carro] = perfil_velocidad_trapezoidal(Ts,0, v_t, x_t_fin, 0, set_param_lim(1), set_param_lim(2));

    % Siempre se parte desde y_max para entrar al barco
    y_max_ini = max(obs) + h_ss;
    pos_y_max = 1;
    for i = 1 : (length(obs) - 1)
        if (obs(i) + h_ss) == y_max_ini
            pos_y_max = i;
        end
    end


    % En caso de que la altura de entrada sea la misma donde dejar el
    % container, se mantiene posicion y devuelve como consigna
    if y_h_fin >= y_max_ini
        t_izaje = 0;
        y_h = y_max_ini;
        dy_h = 0;


        % Encapsulamiento de las consignas
        carro_cons = [x_t; dx_t]';
        izaje_cons = [y_h; dy_h]';

        return;
    end


    % Se encuentran valores en x y en y donde se realiza el izaje
    % escalonado
    pos_x_max = [];
    y_max_vec = [];
    y_max = y_max_ini;

    for i = pos_y_max : length(obs)
        if (obs(i) + h_ss) == y_max
            pos_x_max = [pos_x_max i * w_c];
            y_max = max(obs(i + 1: end)) + h_ss;
            y_max_vec = [y_max_vec y_max];
            if y_max == y_h_fin
                break;
            end
        end
    end

    % Se correlaciona con el movimiento del carro
    t_izaje = 0;
    pos_aux = 1;
    y_h_aux = [y_max_ini]; y_h = [];
    dy_h_aux = [0]; dy_h = [];
    y_max = y_max_ini;

    for idx = 1 : length(x_t)
        if x_t(idx) >= pos_x_max(pos_aux) && (idx * Ts) > t_izaje
            % Se evalua si es el ultimo escalon a descender, y se deja
            % velocidad final en v_max
            if pos_aux == length(pos_x_max)
                [y, dy, t_izaje_aux] = perfil_velocidad_trapezoidal(Ts, y_max, 0, y_max_vec(pos_aux), -set_param_lim(3), set_param_lim(3), set_param_lim(4));
                y_h = [y_h y_h_aux y];
                dy_h = [dy_h dy_h_aux dy];
                disp("A")

                t_izaje = (idx * Ts) + t_izaje_aux;
                break;
            end

            % Una vez superado el obstaculo mas alto se empieza a descender
            [y, dy, t_izaje_aux] = perfil_velocidad_trapezoidal(Ts, y_max, 0, y_max_vec(pos_aux), -set_param_lim(3)/3, set_param_lim(3), set_param_lim(4));
            y_h = [y_h y_h_aux y];
            dy_h = [dy_h dy_h_aux dy];
  
            % Se actualizan auxiliares
            y_max = y_max_vec(pos_aux);
            pos_aux = pos_aux + 1;
            y_h_aux = [y_max];
            dy_h_aux = [0];
            idx_aux = 1;


            % Se traslada el tiempo de izaje a idx
            t_izaje = (idx * Ts) + t_izaje_aux;

        end
        if (idx * Ts) > t_izaje
            y_h_aux(end+1) = y_max;
            dy_h_aux(end+1) = 0;
        end
    end
end


    % Encapsulamiento de las consignas
    carro_cons = [x_t; dx_t]';
    izaje_cons = [y_h; dy_h]';

end

