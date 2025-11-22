function [x, v, t_total] = perfil_velocidad_trapezoidal(Ts, x_0, v_0, x_f, v_f, v_max, a_max)
    % Perfil trapezoidal con velocidades inicial y final arbitrarias.
    % Robustamente estable, sin rebotes, soporta dirección y velocidad invertida.

    % ---- Dirección del movimiento ----
    dir = sign(x_f - x_0);
    if dir == 0
        x = x_0;
        v = v_0;
        t_total = 0;
        return;
    end

    % Normalizar a eje positivo
    x0 = x_0 * dir;
    xf = x_f * dir;
    v0 = v_0 * dir;
    vf = v_f * dir;
    vmax = abs(v_max);

    amax = abs(a_max);

    % Distancia útil
    D = xf - x0;

    if D <= 0
        x = x_0;
        v = v_0;
        t_total = 0;
        return;
    end

    % ---- Corrección inicial: si v0 < 0 → frenar ----
    if v0 < 0
        t_brake = -v0 / amax;
        x0 = x0 + (v0*t_brake + 0.5*(-amax)*t_brake^2);
        v0 = 0;
        D = xf - x0;
        if D <= 0
            x = x_0;
            v = 0;
            t_total = 0;
            return;
        end
    end

    % Si v0 > vmax → frenar hasta vmax
    if v0 > vmax
        t_brake = (v0 - vmax)/amax;
        x0 = x0 + (v0*t_brake - 0.5*amax*t_brake^2);
        v0 = vmax;
        D = xf - x0;
        if D <= 0
            x = x_0;
            v = v_max * dir;
            t_total = 0;
            return;
        end
    end

    % ---- Frenado final requerido a vf ----
    if vf < 0
        % vf negativo no tiene sentido en eje positivo → se forzará a 0
        vf = 0;
    end
    if vf > vmax
        vf = vmax;
    end

    % Distancia para frenar desde vmax hasta vf
    t_dec = (vmax - vf) / amax;
    x_dec = vmax*t_dec - 0.5*amax*t_dec^2;

    % Distancia para acelerar de v0 a vmax
    t_acc = (vmax - v0) / amax;
    x_acc = v0*t_acc + 0.5*amax*t_acc^2;

    % Distancia restante para velocidad constante
    x_const = D - x_acc - x_dec;

    % ---- Caso triangular (no alcanza vmax) ----
    if x_const < 0
        % Resolver velocidad pico VP que cumple:
        % VP^2 - v0^2 = 2*a*(distancia disponible menos frenado final)
        % Pero ahora el frenado final debe dejar velocidad vf
        % Fórmula general:
        % (VP^2 - v0^2)/(2a) + (VP^2 - vf^2)/(2a) = D
        A = 1;
        B = 0;
        C = -(v0^2 + vf^2 + 2*amax*D);

        VP = sqrt(max((v0^2 + vf^2 + 2*amax*D)/2,0));

        % Ajustar fases
        t_acc = (VP - v0)/amax;
        x_acc = v0*t_acc + 0.5*amax*t_acc^2;

        t_dec = (VP - vf)/amax;
        x_dec = VP*t_dec - 0.5*amax*t_dec^2;

        t_const = 0;
        vmax = VP;
    else
        % ---- Perfil trapezoidal ----
        t_const = x_const / vmax;
    end

    % Tiempo total
    t_total = t_acc + t_const + t_dec;

    % ---- Construcción del perfil ----
    t = 0:Ts:t_total;
    N = length(t);
    x = zeros(1, N);
    v = zeros(1, N);

    x_start_dec = x0 + x_acc + vmax*t_const;

    for i = 1:N
        ti = t(i);

        if ti <= t_acc + 1e-12
            % Aceleración desde v0
            v(i) = v0 + amax*ti;
            x(i) = x0 + v0*ti + 0.5*amax*ti^2;

        elseif ti <= t_acc + t_const + 1e-12
            % Velocidad constante
            tc = ti - t_acc;
            v(i) = vmax;
            x(i) = x0 + x_acc + vmax*tc;

        else
            % Desaceleración hasta vf
            td = ti - (t_acc + t_const);
            v(i) = vmax - amax*td;
            if v(i) < vf
                v(i) = vf;
            end
            x(i) = x_start_dec + vmax*td - 0.5*amax*td^2;
        end
    end

    % Desnormalizar al eje real
    x = x * dir;
    v = v * dir;
end
