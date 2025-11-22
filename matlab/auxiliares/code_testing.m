clc;



set_param_limite = [dx_t_lim ddx_t_lim dy_h_lim(1) ddy_h_lim];
y_c = [5 0 4 0 2 0 2 3]


% consigna_en_barco(Ts, w_c, h_c, perfil_obs, set_param_lim, p_ini, p_fin, h_ss, v_t, v_h, tipo)
[carro, izaje, t_carro, t_izaje] = consigna_en_barco(T_s1, W_c, H_c, y_c, set_param_limite, 0, 8, 0.5, 0, 0, 1);


t_izaje / T_s1
length(izaje)
t_carro / T_s1
length(carro)






figure("Name","Izaje")
th = 0:T_s1:t_izaje - T_s1;
subplot(2,1,1);
plot(th, izaje(:,1));
subplot(2,1,2);
plot(th, izaje(:,2));

figure("Name","Carro")
tc = 0:T_s1:t_carro;
subplot(2,1,1);
plot(tc, carro(:,1));
subplot(2,1,2);
plot(tc, carro(:,2));