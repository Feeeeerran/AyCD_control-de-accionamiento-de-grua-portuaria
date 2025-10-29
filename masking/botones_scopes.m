classdef botones_scopes

    methods(Static)
        
        function btn1(~)
            open_system(strcat(gcb,"/Torques de accionamientos"))
        end

        function btn2(~)
            open_system(strcat(gcb,"/Posicion de carro e izaje"))
        end

        function btn3(~)
            open_system(strcat(gcb,"/Coordenadas de cable"))
        end

        function btn4(~)
            open_system(strcat(gcb,"/Angulo de cable"))
        end
    end
end

