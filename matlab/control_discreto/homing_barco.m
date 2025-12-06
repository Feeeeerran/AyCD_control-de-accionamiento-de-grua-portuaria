function [carro_cons, izaje_cons, BRK_t] = homing_barco(Ts, perfil_obstaculos, w_c, param_lim, h_ss, tipo, p_barco, v_h_ini)
% Se generan las consignas para el Homing en zona de Barco con trayectoria optima.
% El tipo de accion depende del parametro tipo:
%   0 - Ir a Home (Desde p_barco -> x=0, y=y_max)
%   1 - Ir a Posicion (Desde x=0, y=y_max -> p_barco)
% Nota: Se asume sin carga (TLK = false), por lo que se usa h_ss estandar.

% --- 1. CONFIGURACIÓN ---
% Posición X de la columna objetivo/origen en el barco
x_col = (p_barco * w_c) - (w_c / 2);

% Altura local (en la columna p_barco)
y_col = perfil_obstaculos(p_barco) + h_ss * 1.5;

% Altura Global (HOME): Máximo de todo el barco + seguridad
y_max_global = max(perfil_obstaculos) + h_ss;

% --- 2. LÓGICA SEGÚN TIPO ---

if tipo == 0
    % =========================================================
    % TIPO 0: IR A HOME (Subir y Moverse a X=0)
    % =========================================================

    % Generación de perfiles base ideales
    % Izaje: Sube de y_col a y_max_global
    [y_up, dy_up, ~] = perfil_velocidad_trapezoidal(Ts, y_col, v_h_ini, y_max_global, 0, param_lim(3), param_lim(4));

    % Carro: Se mueve de x_col a 0
    [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, x_col, 0, 0, 0, param_lim(1), param_lim(2));

    % Optimización: Buscar retraso del carro para evitar colisión a la izquierda
    delay_carro = 0;
    retraso = 0;

    % Se revisan columnas desde p_barco hacia la izquierda (hasta la 1)
    for k = p_barco:-1:1
        x_borde = (k - 1) * w_c;
        h_req = perfil_obstaculos(k) + h_ss;

        % Se busca intersección de tiempos
        idx_cruce_carro = find(x_mov <= x_borde, 1, 'first');
        if isempty(idx_cruce_carro), continue; end

        idx_seguro_izaje = find(y_up >= h_req, 1, 'first');
        if isempty(idx_seguro_izaje), idx_seguro_izaje = length(y_up); end

        retraso = idx_seguro_izaje - idx_cruce_carro;


        if retraso > delay_carro
            delay_carro = retraso(1);
        end
    end
    delay_carro = max(0, delay_carro);

    % Ensamblaje de vectores
    carro_pos = [ones(delay_carro, 1) * x_col; x_mov'];
    carro_vel = [zeros(delay_carro, 1); dx_mov'];

    % Izaje simplemente sube y se mantiene en Home
    L_carro = length(carro_pos);
    L_izaje = length(y_up);
    L_total = max(L_carro, L_izaje);

    % Relleno de vectores para igualar longitudes
    if L_carro < L_total
        dif = L_total - L_carro;
        carro_pos = [carro_pos; ones(dif,1)*0];
        carro_vel = [carro_vel; zeros(dif,1)];
    end

    if L_izaje < L_total
        dif = L_total - L_izaje;
        izaje_pos = [y_up'; ones(dif,1) * y_max_global];
        izaje_vel = [dy_up'; zeros(dif,1)];
    else
        izaje_pos = y_up'; izaje_vel = dy_up';
    end

else
    % =========================================================
    % TIPO 1: IR A POSICIÓN (Salir de Home -> Bajar entre containers)
    % =========================================================

    % Generación de perfiles base ideales
    % Carro: De 0 a x_col
    [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, 0, 0, x_col, 0, param_lim(1), param_lim(2));

    % Izaje: De y_max_global a y_col (Bajada)
    [y_dn, dy_dn, ~] = perfil_velocidad_trapezoidal(Ts, y_max_global, 0, y_col, v_h_ini, param_lim(3), param_lim(4));

    L_carro = length(x_mov);
    L_bajada = length(y_dn);

    % Optimización: Buscar inicio de bajada más temprano posible
    % Se intenta bajar desde el instante 0 (Diagonal agresiva)
    start_dn_idx = 1;

    while true
        collision = false;

        for i = 1:L_bajada
            idx_global = start_dn_idx + i - 1;

            % Posición estimada del carro
            if idx_global <= L_carro
                x_now = x_mov(idx_global);
            else
                x_now = x_col;
            end

            y_now = y_dn(i);

            % Chequeo de colisión (Obstáculos a la derecha)
            if x_now > 0
                col = ceil(x_now / w_c);
                if col >= 1 && col <= length(perfil_obstaculos)
                    h_obs = perfil_obstaculos(col);
                    if y_now < (h_obs + h_ss - 0.05)
                        collision = true;
                        break;
                    end
                end
            end
        end

        if collision
            start_dn_idx = start_dn_idx + 1;
        else
            break;
        end
    end

    % Ensamblaje de vectores
    carro_pos = x_mov';
    carro_vel = dx_mov';

    % Izaje espera en Home hasta start_dn_idx
    wait_steps = start_dn_idx - 1;
    if wait_steps < 0, wait_steps = 0; end

    izaje_pos = [ones(wait_steps, 1)*y_max_global; y_dn'];
    izaje_vel = [zeros(wait_steps, 1); dy_dn'];

    % Igualar longitudes
    L_total = max(length(carro_pos), length(izaje_pos));

    if length(carro_pos) < L_total
        dif = L_total - length(carro_pos);
        carro_pos = [carro_pos; ones(dif,1)*x_col];
        carro_vel = [carro_vel; zeros(dif,1)];
    end

    if length(izaje_pos) < L_total
        dif = L_total - length(izaje_pos);
        izaje_pos = [izaje_pos; ones(dif,1)*y_col];
        izaje_vel = [izaje_vel; zeros(dif,1)];
    end
end

% Encapsulamiento de las consignas
carro_cons = [carro_pos, carro_vel];
izaje_cons = [izaje_pos, izaje_vel];
BRK_t = L_total * Ts;
end