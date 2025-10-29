clc;

%% Especificaciones para el autómata
% Limites de traslacion iniciales
    % El operador ajusta, mediante HMI los siguientes parametros
    % Sobre muelle (0 a -30m)
        % Limites de traslacion iniciales
        % Carriles fijos (carga o descarga)
    % Sobre barco
        % Posicion objetivo fija / incremental

% Altura de izaje
    % Maxima inicial (despeje de obstaculos, viga testera, contenedores, etc)
    % Minimas inicial de izaje
    % Final de descenso automatico


%% Perfil de obstaculos
% El usuario especifica la cantidad filas en el barco para posicionar los
% containers, en base a este numero (siempre y cuando no se superen los
% límites) se calculan los nuevos limites

% A modo de pruebas, se especifican 10 filas de contenedores
filas_contenedores = 10;

% Se genera vector que  tendra la informacion de la cantidad de
% contenedores apilados en cada fila
perfil_obstaculos = zeros(1,filas_contenedores);

% Se determina el nuevo limite operativo (si respeta el lim. real)
x_lim_operativo = filas_contenedores * W_c;


% Se rellena el perfil de obscatulos con containers
perfil_obstaculos = randi([0, 6], 1, 10);
