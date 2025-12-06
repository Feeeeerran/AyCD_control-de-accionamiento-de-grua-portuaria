function [carro_cons, izaje_cons, t_freno_carro, t_freno_izaje] = cambio_muelle(Ts, w_c, h_c, param_lim, h_ss, n_muelle_ini, n_muelle_fin, v_h_ini)
% Se generan las consignas para mover el carro de una zona de muelle a otra.
% Operación exclusiva en zona de muelle (sin obstáculos aéreos).
% Se asume TLK = false (sin carga), por lo que se usa h_ss estándar.
%
% Argumentos extra:
%   v_h_ini: Velocidad inicial del izaje (permite estabilizar si trae inercia).

    % --- 1. CONFIGURACIÓN ---
    % Cálculo de posiciones X inicial y final
    x_ini = -((n_muelle_ini * 2 * w_c) - (w_c / 2));
    x_fin = -((n_muelle_fin * 2 * w_c) - (w_c / 2));
    
    % Altura de seguridad para transporte en muelle
    y_transporte = h_c + h_ss;

    % --- 2. GENERACIÓN DE PERFILES BASE ---
    
    % Movimiento del carro: Recorrido directo de x_ini a x_fin
    [x_mov, dx_mov, ~] = perfil_velocidad_trapezoidal(Ts, x_ini, 0, x_fin, 0, param_lim(1), param_lim(2));
    
    % Movimiento del izaje: Estabilización en altura de transporte
    % Aunque la altura es la misma, usamos la función para frenar suavemente si v_h_ini != 0
    [y_mov, dy_mov, ~] = perfil_velocidad_trapezoidal(Ts, y_transporte, v_h_ini, y_transporte, -param_lim(3), param_lim(3), param_lim(4));
    
    % --- 3. ENSAMBLAJE Y SINCRONIZACIÓN ---
    
    L_carro = length(x_mov);
    L_izaje = length(y_mov);
    L_total = max(L_carro, L_izaje);
    
    carro_pos = x_mov'; carro_vel = dx_mov';
    izaje_pos = y_mov'; izaje_vel = dy_mov';
    
    % Relleno Carro
    if L_carro < L_total
        dif = L_total - L_carro;
        carro_pos = [carro_pos; ones(dif, 1) * x_fin];
        carro_vel = [carro_vel; zeros(dif, 1)];
    end
    
    % Relleno Izaje
    if L_izaje < L_total
        dif = L_total - L_izaje;
        izaje_pos = [izaje_pos; ones(dif, 1) * y_transporte];
        izaje_vel = [izaje_vel; zeros(dif, 1)];
    end
    
    % Encapsulamiento
    carro_cons = [carro_pos, carro_vel];
    izaje_cons = [izaje_pos, izaje_vel];
    
    % Cálculo de tiempos de frenado independientes
    t_freno_carro = L_carro * Ts;
    t_freno_izaje = L_izaje * Ts; % Será casi 0 si v_h_ini es 0
end