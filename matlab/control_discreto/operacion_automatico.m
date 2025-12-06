function [carro_cons, izaje_cons, BRK_t, BRK_h] = operacion_automatico(Ts, perfil_obstaculos, w_c, h_c, TLK, param_lim, h_ss, tipo, n_muelle, p_barco_ini, p_barco_fin, v_h0, v_hf)
% Se generan las consignas para la operación automática de carga/descarga o movimiento interno.
% El tipo de accion depende del parametro tipo:
%   0 - Descarga (Barco -> Muelle)
%   1 - Carga (Muelle -> Barco)
%   2 - Movimiento Interno (Barco Columna A -> Barco Columna B)
%
% Parametros Nuevos:
%   p_barco_fin: (Opcional/Requerido para tipo 2) Columna destino dentro del barco.

    % --- 0. VALIDACIÓN DE ARGUMENTOS ---
    if nargin < 13
        p_barco_fin = p_barco_ini; % Valor por defecto si no se usa tipo 2
    end

    % --- 1. CONFIGURACIÓN ---
    
    % Ajuste de altura de seguridad si hay carga (TLK activado)
    h_ss_ajustada = h_ss * (1 + double(TLK));
    
    % Posición y Altura Inicial (Común para todos)
    x_ini = (p_barco_ini * w_c) - (w_c / 2);
    y_ini = perfil_obstaculos(p_barco_ini) + h_ss_ajustada; 

    % CONFIGURACIÓN SEGÚN TIPO DE OPERACIÓN
    if tipo == 2
        % === TIPO 2: MOVIMIENTO INTERNO (Barco -> Barco) ===
        x_fin = (p_barco_fin * w_c) - (w_c / 2);
        
        % La altura final es la altura de la columna destino + seguridad
        y_final = perfil_obstaculos(p_barco_fin) + h_ss_ajustada;
        
        % Rango de columnas a evaluar para obstáculos
        col_min = min(p_barco_ini, p_barco_fin);
        col_max = max(p_barco_ini, p_barco_fin);
        cols_recorrido = col_min:col_max;
        
        % Dirección del análisis de colisiones (hacia dónde nos movemos)
        if p_barco_fin > p_barco_ini
            dir_analisis = 1; % Derecha
            col_limite = p_barco_fin;
        else
            dir_analisis = -1; % Izquierda
            col_limite = p_barco_fin;
        end
        
    else
        % === TIPO 0/1: OPERACIÓN MUELLE (Barco <-> Muelle) ===
        % Calculamos siempre como Barco -> Muelle (Tipo 0) y luego invertimos si es Tipo 1
        x_fin = -((n_muelle * 2 * w_c) - (w_c / 2));
        y_final = h_c + h_ss; % Altura sobre camión
        
        % Rango de columnas (desde barco hasta el borde del barco)
        cols_recorrido = 1:p_barco_ini;
        
        % Dirección siempre es hacia la izquierda (hacia muelle)
        dir_analisis = -1; 
        col_limite = 1;
        
        % Si es carga (Tipo 1), invertimos velocidades de consigna manual
        if tipo == 1
            v_aux = v_h0; v_h0 = v_hf; v_hf = v_aux;
        end
    end
    
    % Calculo de altura de crucero
    % Debe ser el máximo obstáculo en el trayecto + seguridad
    y_max_recorrido = max([perfil_obstaculos(cols_recorrido), 0]); 
    y_crucero = y_max_recorrido + h_ss_ajustada;
    
    % --- 2. GENERACIÓN DE PERFILES BASE ---
    % Movimiento de izaje: Subida
    [y_up, dy_up, ~] = perfil_velocidad_trapezoidal(Ts, y_ini, v_h0, y_crucero, 0, param_lim(3), param_lim(4));
    
    % Movimiento de izaje: Bajada
    [y_dn, dy_dn, ~] = perfil_velocidad_trapezoidal(Ts, y_crucero, 0, y_final, -v_hf, param_lim(3), param_lim(4));
    
    % Movimiento del carro: Recorrido completo
    [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, x_ini, 0, x_fin, 0, param_lim(1), param_lim(2));

    % --- 3. OPTIMIZACIÓN DE SALIDA (Delay Carro) ---
    delay_carro_start = 0;
    
    % Se evaluan las columnas en la dirección del movimiento
    % Desde p_barco hacia el destino
    % col_iter = p_barco : dir_analisis : col_limite;
    
    for k = p_barco_ini : dir_analisis : col_limite
        % Definición de bordes de la columna k
        % Si vamos a la derecha, nos preocupa el borde derecho ((k)*w), si izq, el borde izq ((k-1)*w)
        % Para simplificar (y por seguridad), chequeamos AMBOS bordes o el centro.
        % Aquí usamos lógica de borde entrante según dirección.
        
        if dir_analisis == 1 % Derecha
            x_borde_check = (k) * w_c; 
            % Buscamos cuando x_mov >= borde (cruce hacia derecha)
            idx_cruce_carro = find(x_mov >= x_borde_check, 1, 'first');
        else % Izquierda
            x_borde_check = (k - 1) * w_c;
            % Buscamos cuando x_mov <= borde (cruce hacia izquierda)
            idx_cruce_carro = find(x_mov <= x_borde_check, 1, 'first');
        end
        
        if isempty(idx_cruce_carro), continue; end
        
        h_req = perfil_obstaculos(k) + h_ss_ajustada;
        
        idx_seguro_izaje = find(y_up >= h_req, 1, 'first');
        if isempty(idx_seguro_izaje), idx_seguro_izaje = length(y_up); end
        
        retraso = idx_seguro_izaje - idx_cruce_carro;
        if retraso > delay_carro_start
            delay_carro_start = retraso(1);
        end
    end
    delay_carro_start = max(0, delay_carro_start);

    % Construcción vector carro
    carro_pos = [ones(delay_carro_start, 1) * x_ini; x_mov'];
    carro_vel = [zeros(delay_carro_start, 1); dx_mov'];
    L_carro = length(carro_pos);

    % --- 4. OPTIMIZACIÓN DE LLEGADA (Delay Bajada) ---
    L_bajada = length(y_dn);
    L_subida = length(y_up);
    
    start_dn_ideal = L_carro - L_bajada + 1;
    start_dn_idx = max(L_subida + 1, start_dn_ideal);
    
    while true
        len_crucero = start_dn_idx - L_subida - 1;
        if len_crucero < 0, len_crucero = 0; end 
        
        collision = false;
        
        for i = 1:length(y_dn)
            idx_global = start_dn_idx + i - 1;
            
            if idx_global <= L_carro
                x_now = carro_pos(idx_global);
            else
                x_now = x_fin;
            end
            y_now = y_dn(i);
            
            % CHEQUEO DE COLISIÓN
            % Para Tipo 0/1: Solo si x > 0 (Zona Barco)
            % Para Tipo 2: Siempre (Todo es Zona Barco)
            
            check_condition = (x_now > 0); 
            if tipo == 2, check_condition = true; end % En tipo 2 siempre chequeamos
            
            if check_condition
                col = ceil(x_now / w_c);
                % Validación de límites del array
                if col >= 1 && col <= length(perfil_obstaculos)
                    h_obs = perfil_obstaculos(col);
                    if y_now < (h_obs + h_ss_ajustada - 0.05)
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

    % --- 5. ENSAMBLAJE FINAL ---
    
    % Construcción Izaje
    len_crucero_final = start_dn_idx - L_subida - 1;
    if len_crucero_final < 0, len_crucero_final = 0; end
    
    izaje_pos = [y_up'; ones(len_crucero_final, 1) * y_crucero; y_dn'];
    izaje_vel = [dy_up'; zeros(len_crucero_final, 1); dy_dn'];
    
    % Sincronización
    L_total = max(length(carro_pos), length(izaje_pos));
    
    % Relleno Carro
    if length(carro_pos) < L_total
        dif = L_total - length(carro_pos);
        carro_pos = [carro_pos; ones(dif, 1) * x_fin];
        carro_vel = [carro_vel; zeros(dif, 1)];
    end
    
    % Relleno Izaje
    if length(izaje_pos) < L_total
        dif = L_total - length(izaje_pos);
        izaje_pos = [izaje_pos; ones(dif, 1) * y_final];
        izaje_vel = [izaje_vel; zeros(dif, 1)];
    end
    
    % Inversión para Tipo 1 (Muelle -> Barco)
    if tipo == 1
        carro_pos = flip(carro_pos);
        carro_vel = flip(carro_vel * -1);
        izaje_pos = flip(izaje_pos);
        izaje_vel = flip(izaje_vel * -1);
    end
    
    carro_cons = [carro_pos, carro_vel];
    izaje_cons = [izaje_pos, izaje_vel];
    
    t_freno_carro = L_carro * Ts;
    t_freno_izaje = L_total * Ts; 
    
    BRK_t = t_freno_carro;
    BRK_h = t_freno_izaje;
end