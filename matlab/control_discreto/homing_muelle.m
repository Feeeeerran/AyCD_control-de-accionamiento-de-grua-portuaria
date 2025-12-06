function [carro_cons, izaje_cons, BRK_t] = homing_muelle(Ts, perfil_obstaculos, w_c, h_c, param_lim, h_ss, tipo, n_muelle, v_h_ini)
% Se generan las consignas para el Homing en zona de Muelle con trayectoria directa.
% El tipo de accion depende del parametro tipo:
%   0 - Ir a Home (Desde muelle -> x=0, y=y_max)
%   1 - Ir a Posicion (Desde x=0, y=y_max -> muelle)
% Nota: Se asume sin carga (TLK = false).

    % --- 1. CONFIGURACIÓN ---
    % Posición X en muelle
    x_muelle = -((n_muelle * 2 * w_c) - (w_c / 2));
    
    % Altura de seguridad (Destino en muelle)
    y_muelle = h_c + h_ss * 1.5;
    
    % Altura Home (Máxima del barco + seguridad)
    y_max_global = max(perfil_obstaculos) + h_ss;

    % --- 2. GENERACIÓN DE PERFILES ---
    
    if tipo == 0
        % =========================================================
        % TIPO 0: IR A HOME (Muelle -> 0, y_max)
        % =========================================================
        
        % Perfil Carro: x_muelle -> 0
        [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, x_muelle, 0, 0, 0, param_lim(1), param_lim(2));
        
        % Perfil Izaje: y_muelle -> y_max_global
        [y_mov, dy_mov, ~] = perfil_velocidad_trapezoidal(Ts, y_muelle, v_h_ini, y_max_global, 0, param_lim(3), param_lim(4));
        
        % Destinos finales para relleno
        x_target = 0;
        y_target = y_max_global;
        
    else
        % =========================================================
        % TIPO 1: SALIR DE HOME (0, y_max -> Muelle)
        % =========================================================
        
        % Perfil Carro: 0 -> x_muelle
        [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, 0, 0, x_muelle, 0, param_lim(1), param_lim(2));
        
        % Perfil Izaje: y_max_global -> y_muelle
        [y_mov, dy_mov, ~] = perfil_velocidad_trapezoidal(Ts, y_max_global, 0, y_muelle, v_h_ini, param_lim(3), param_lim(4));
        
        x_target = x_muelle;
        y_target = y_muelle;
    end
    
    % --- 3. ENSAMBLAJE (DIAGONAL PURA) ---
    % Se ejecutan ambos movimientos simultáneamente.
    
    L_carro = length(x_mov);
    L_izaje = length(y_mov);
    L_total = max(L_carro, L_izaje);
    
    carro_pos = x_mov'; carro_vel = dx_mov';
    izaje_pos = y_mov'; izaje_vel = dy_mov';
    
    % Relleno Carro
    if L_carro < L_total
        dif = L_total - L_carro;
        carro_pos = [carro_pos; ones(dif,1)*x_target];
        carro_vel = [carro_vel; zeros(dif,1)];
    end
    
    % Relleno Izaje
    if L_izaje < L_total
        dif = L_total - L_izaje;
        izaje_pos = [izaje_pos; ones(dif,1)*y_target];
        izaje_vel = [izaje_vel; zeros(dif,1)];
    end

    % Encapsulamiento de las consignas
    carro_cons = [carro_pos, carro_vel];
    izaje_cons = [izaje_pos, izaje_vel];
    BRK_t = L_total*Ts;
end