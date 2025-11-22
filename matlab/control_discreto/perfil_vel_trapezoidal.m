
% Testeando el generador de perfil de velocidad trapezoidal

T_s = 0.01;
x_f = 20;
v_max = 1;
a_max = 0.5;

[t, x, v, a] = perfil_velocidad_trapezoidal(T_s, x_f, v_max, a_max);

figure;
subplot(3,1,1); plot(t, x); ylabel('Posición [m]');
subplot(3,1,2); plot(t, v); ylabel('Velocidad [m/s]');
subplot(3,1,3); plot(t, a); ylabel('Aceleración [m/s^2]'); xlabel('Tiempo [s]');




function [t, x, v, a] = perfil_velocidad_trapezoidal(T_s, x_f, v_max, a_max)
%   Siendo
%   T_s     tiempo de muestreo [s]
%   x_f     distancia final [m]
%   v_max   velocidad máxima [m/s]
%   a_max   aceleración máxima [m/s^2]
%
% Salidas:
%   t -> vector de tiempo [s]
%   x -> posición [m]
%   v -> velocidad [m/s]
%   a -> aceleración [m/s^2]

    % Tiempo de aceleración y distancia de aceleración
    t_acc = v_max / a_max;
    x_acc = 0.5 * a_max * t_acc^2;

    % Dada la distancia a recorrer, se puede alcanzar la velocidad maxima,
    % lo que genera un perfil trapezoidal de velocidad, o no alcanzar dicha
    % velocidad y generar un perfil de velocidad triangular
    if 2 * x_acc > x_f
        % Perfil triangular
        t_acc = sqrt(x_f / a_max);
        t_const = 0;
        t_total = 2 * t_acc;
        v_max = a_max * t_acc;
    else
        % Perfil triangular
        x_const = x_f - 2 * x_acc;
        t_const = x_const / v_max;
        t_total = 2 * t_acc + t_const;
    end

    % Vector de tiempo
    t = 0:T_s:t_total;

    % Para comparar con la salida del sistema se generan valores de
    % posicion y aceleracion
    x = zeros(size(t));
    v = zeros(size(t));
    a = zeros(size(t));

    % Generación de datos
    for i = 1:length(t)
        ti = t(i);
        if ti <= t_acc
            % Aceleracion
            a(i) = a_max;
            v(i) = a_max * ti;
            x(i) = 0.5 * a_max * ti^2;
        elseif ti <= (t_acc + t_const)
            % Velocidad constante (v_max)
            a(i) = 0;
            v(i) = v_max;
            x(i) = x_acc + v_max * (ti - t_acc);
        else
            % Desaceleracion
            td = ti - (t_acc + t_const);
            a(i) = -a_max;
            v(i) = v_max - a_max * td;
            x(i) = x_f - 0.5 * a_max * (t_total - ti)^2;
        end
    end
end
