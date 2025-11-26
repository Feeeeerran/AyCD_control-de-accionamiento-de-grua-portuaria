clc;



set_param_lim = [dx_t_lim ddx_t_lim dy_h_lim(2) ddy_h_lim];

% y_max = (max(y_c(1:5)) * H_c) + H_SS;
% 
% % consigna_en_barco(Ts, w_c, h_c, perfil_obs, set_param_lim, p_ini, p_fin, h_ss, v_t, v_h, tipo)
% % consigna_en_muelle(Ts, w_c, set_param_lim, n_muelle, altura_izaje, h_ss, v_t, v_h, tipo)
% 
% calc_aux = (CI(3) - H_SS) / H_c;
% 
% [carro, izaje, t_carro, t_izaje] = consigna_en_barco(T_s1, W_c, H_c, [calc_aux y_c(2:end)], set_param_lim, 0, 5, H_SS, 0, 0, 1);
% 


[x, dx, t] = perfil_velocidad_trapezoidal(T_s1, 0, 0, (50 - W_c/2),0, set_param_lim(3), set_param_lim(4))

x = flip(x);
dx = flip(dx);

t = 0:T_s1:t;
subplot(2,1,1);
plot(t,x);
subplot(2,1,2);
plot(t, dx);




% figure("Name","Izaje","Position",[200 200 500 500])
% th = 0:T_s1:t_izaje;
% subplot(2,1,1);
% plot(th, izaje(:,1));
% subplot(2,1,2);
% plot(th, izaje(:,2));
% 
% figure("Name","Carro","Position",[200 200 500 500])
% tc = 0:T_s1:t_carro;
% subplot(2,1,1);
% plot(tc, carro(:,1));
% subplot(2,1,2);
% plot(tc, carro(:,2));