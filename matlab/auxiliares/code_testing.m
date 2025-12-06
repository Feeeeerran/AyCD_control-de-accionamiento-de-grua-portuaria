clc
% SCRIPT: prueba_operacion_barco_a_muelle_final.m
% NOTA: Necesita la función 'perfil_velocidad_trapezoidal' operativa.


y_c_test = [2 4 3 2 1 0 0 0] * H_c; % Perfil de obstaculos (altura absoluta)
TLK = false;         % Con carga
n_muelle = 1;       % Zona de muelle (distancia)
p_barco = 5;        % Posicion de la pila en el barco

set_param_lim = [dx_t_lim ddx_t_lim dy_h_lim(1) ddy_h_lim];

% --- 2. LLAMADA A LA FUNCIÓN ---
disp('Generando consignas para Operacion Barco a Muelle...');
% [carro_cons, izaje_cons, BRK_t] = operacion_automatico2(T_s1, y_c_test, W_c, H_c, TLK, set_param_lim, H_SS, 2, n_muelle, p_barco, p_barco, 0, 0)
% [carro_cons, izaje_cons, BRK_t] = homing_muelle(T_s1, y_c_test, W_c, H_c, set_param_lim, H_SS, 0, n_muelle, 0)
% [carro_cons, izaje_cons, BRK_t] = homing_barco(T_s1, y_c_test, W_c, set_param_lim, H_SS, 1, p_barco, -0.5);
[carro_cons, izaje_cons, t_freno_carro, t_freno_izaje] = cambio_muelle(T_s1, W_c, H_c, set_param_lim, H_SS, 1, 5, 0)

size(carro_cons)
size(izaje_cons)










graph = 1;

if graph == 1

    % --- 3. GRÁFICO SIMPLE DE RESULTADOS ---
    N = size(carro_cons, 1);
    t = (0:N-1) * T_s1;

    figure('Name', 'Consignas Barco a Muelle');
    subplot(2, 1, 1);
    plot(t, carro_cons(:, 1), 'b', 'LineWidth', 2);
    title('Posición del Carro (x_t)');
    xlabel('Tiempo [s]'); ylabel('Posición [m]');
    grid on;

    subplot(2, 1, 2);
    plot(t, izaje_cons(:, 1), 'r', 'LineWidth', 2);
    title('Posición del Izaje (y_h)');
    xlabel('Tiempo [s]'); ylabel('Posición [m]');
    grid on;

    figure('Name', 'Consignas Barco a Muelle vel');
    subplot(2, 1, 1);
    plot(t, carro_cons(:, 2), 'b', 'LineWidth', 2);
    title('Velocidad del Carro (x_t)');
    xlabel('Tiempo [s]'); ylabel('Posición [m]');
    grid on;

    subplot(2, 1, 2);
    plot(t, izaje_cons(:, 2), 'r', 'LineWidth', 2);
    title('Velocidad del Izaje (y_h)');
    xlabel('Tiempo [s]'); ylabel('Posición [m]');
    grid on;


    % --- 3. GRÁFICA DE VALIDACIÓN (2D) ---
    figure('Color', 'w'); hold on; axis equal; grid on;

    % Dibujar obstáculos (Usando y_c_test directamente)
    for i = 1:length(y_c_test)
        x_left = (i - 1) * W_c;
        x_right = i * W_c;
        y_top = y_c_test(i); % Ya está en metros

        if y_top > 0
            fill([x_left, x_right, x_right, x_left], [0, 0, y_top, y_top], ...
                [0.7 0.7 0.7], 'EdgeColor', 'k');
        end
    end

    % Trayectoria
    plot(carro_cons(:,1), izaje_cons(:,1), 'b-', 'LineWidth', 2);
    plot(carro_cons(1,1), izaje_cons(1,1), 'go', 'MarkerFaceColor', 'g'); % Inicio
    plot(carro_cons(end,1), izaje_cons(end,1), 'ro', 'MarkerFaceColor', 'r'); % Fin

    title('Verificación Visual de la Maniobra');
    xlabel('Posición Carro [m]'); ylabel('Altura Izaje [m]');
end


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



