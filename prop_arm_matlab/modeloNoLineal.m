% ========== arm_propeller_NL_0p5s_delay_to_90deg.m ==========
% Modelo NO lineal (sin cambios), PWM digital, 4 entradas (Paso/Rampa/Parábola/Seno)
% Requisitos:
%  - Retardo de 0.5 s (entrada=0 hasta t=0.5 s)
%  - Subir desde 0 hasta 90° (pi/2 rad) y detener simulación al cruzar pi/2
%  - 4 figuras separadas con los 5 subplots (v_ref, PWM digital, Vpwm_avg vs v_emf, omega_a, theta)
%  - Unidades: theta [rad], omega [rad/s], voltajes [V]

close all; clear; clc;

%% ---------------- Parámetros físicos (tuyos) ----------------
p = struct();
p.Ke = 5.5e-3;        % [V/(rad/s)]
p.Km = 5.5e-3;        % [N*m/A]
p.Jm = 3e-6;          % [kg*m^2]
p.La1 = 0.21; p.La2 = 0.21;  % [m]
p.Rm = 1;  p.Rs = 1;         % [ohm]
p.Kf = 10e-6;                % [N*m*s]
p.g  = 9.81;                 % [m/s^2]
p.Mm = 0.06;                 % [kg]

% Momento de inercia del brazo (consistente con L^2)
p.m1=0.014; p.m2=0.014; p.mh1=0.016; p.mh2=0.016;
p.La = p.La1;
p.Ja = (1/3)*p.m1*p.La1^2 + p.mh1*p.La1^2 + (1/3)*p.m2*p.La2^2 + p.mh2*p.La2^2;

% Rozamientos del brazo
p.b_arm = 5e-3;       % [N*m*s]
p.c_aero = 0;         % [N*m/(rad/s)^2]

% Empuje cuadrático: f_m = (kT/Ke^2) * v_emf^2  (ajuste a ~300 rad/s)
omega_ref     = 300;           % [rad/s]
Kt_arm_spec   = 1.8e-3;        % [N*s/rad] (pendiente empuje a omega_ref)
kT            = Kt_arm_spec/(2*omega_ref);
p.kT_over_Ke2 = kT/p.Ke^2;     % [N/V^2]

% Fuente y PWM
Vbus = 11;                     % [V] (3S ~ 11 V nominal)
f_pwm = 500;                   % [Hz] PWM digital para visualización
use_switching_in_ode = false;  % true => integra con PWM 0/Vbus (más rígido sin Lm)
duty_max = 0.95;               % saturación de duty

%% --------- Punto de equilibrio aproximado (informativo) ----------
alpha_fm = p.kT_over_Ke2;                         % [N/V^2]
v_emf0   = sqrt((p.Mm*p.g)/alpha_fm);            % [V] para theta=0 horizontal
v_pwm0   = v_emf0*( 1 + p.Kf*(p.Rs+p.Rm)/(p.Km*p.Ke) );
duty_eq  = min(max(v_pwm0/Vbus,0),1);
fprintf('Equilibrio aprox: v_emf0=%.3f V, v_pwm0=%.3f V, duty_eq=%.3f\n',v_emf0,v_pwm0,duty_eq);

%% ---------------- Programación temporal y delay ----------------
T_delay = 0.5;     % [s] *** RETARDO: entrada=0 hasta t=0.5 s ***
T_up    = 1.2;     % [s] duración del tramo de subida (luego puede seguir 1)
T_total = T_delay + T_up + 1.0;   % ventana total de simulación

% Duty de base (antes de iniciar): 0 para cumplir "inicia en 0"
duty_low = 0.0;

% Objetivo angular y evento de parada
theta_target = pi/2;  % 90° en rad

%% ---------------- Tipos de entrada y paleta ----------------
types = {'Paso','Rampa','Parabola','Seno'};

pal = struct();
pal.Paso     = struct('cmd',[0.85 0.33 0.10],'pwm',[0.00 0.45 0.74],'avg',[0.47 0.67 0.19],...
                      'w',[0.93 0.69 0.13],'th',[0.49 0.18 0.56],'emf',[0.20 0.62 0.20]);
pal.Rampa    = struct('cmd',[0.12 0.47 0.71],'pwm',[0.58 0.40 0.74],'avg',[0.55 0.76 0.29],...
                      'w',[0.89 0.10 0.11],'th',[0.17 0.63 0.17],'emf',[0.84 0.37 0.00]);
pal.Parabola = struct('cmd',[0.90 0.12 0.34],'pwm',[0.20 0.20 0.70],'avg',[0.30 0.74 0.93],...
                      'w',[0.60 0.31 0.64],'th',[0.99 0.55 0.38],'emf',[0.55 0.83 0.78]);
pal.Seno     = struct('cmd',[0.30 0.30 0.30],'pwm',[0.64 0.08 0.18],'avg',[0.12 0.47 0.71],...
                      'w',[0.17 0.63 0.17],'th',[0.84 0.37 0.00],'emf',[0.58 0.40 0.74]);

%% ---------------- Bucle por cada tipo de entrada ----------------
for k = 1:numel(types)
    entry  = types{k};
    colors = pal.(entry);

    % r_base(t) con DELAY: 0 hasta T_delay; luego forma (Paso/Rampa/Parábola/Seno) en [T_delay, T_delay+T_up]
    r_base = make_shape_with_delay(entry, T_delay, T_up);

    % Calibrar duty_high (<= duty_max) para alcanzar ~pi/2 (90°) durante la subida
    duty_high = calibrate_duty_high(@(dH) simulate_hit_time(dH, duty_low, r_base, ...
                                 Vbus, f_pwm, use_switching_in_ode, p, T_total, theta_target), ...
                                 max(duty_eq*1.02, 0.05), duty_max, theta_target);

    fprintf('[%s] duty_high=%.3f (eq=%.3f)\n', entry, duty_high, duty_eq);

    % Construir funciones de entrada a partir de r_base y (duty_low,duty_high)
    [u_ref, duty_fun, vpwm_avg_fun, vpwm_sw_fun] = make_input_from_shape(r_base, duty_low, duty_high, Vbus, f_pwm);

    % Simulación con evento de parada al cruzar 90° (theta_target)
    x0    = [0;0;0];                 % [theta; omega_a; v_emf]
    tspan = [0 T_total];

    u_fun = vpwm_avg_fun; if use_switching_in_ode, u_fun = vpwm_sw_fun; end
    opts  = odeset('RelTol',1e-7,'AbsTol',1e-9,'Events',@(t,x) event_theta_target(t,x,theta_target));
    [t,X,te,~,~] = ode45(@(t,x) dyn_prop_arm_NL(t,x,u_fun,p), tspan, x0, opts);

    theta  = X(:,1);  omegaa = X(:,2);  vemf = X(:,3);

    % Señales densas de entrada para mostrar el delay claramente
    t_plot = linspace(0, max(t(end),T_total), 4000);
    vref   = u_ref(t_plot);
    vpwm_d = vpwm_sw_fun(t_plot);
    vpwm_a = vpwm_avg_fun(t_plot);

    %% --------- FIGURA SEPARADA (5 subplots) ---------
    figure('Name',['Entrada: ' entry ' (delay 0.5 s, 0->90°)'], ...
           'Position',[80 60 1200 920],'Color','w');

    % (1) Comando promedio v_ref(t) = Vpwm_avg(t)
    subplot(5,1,1);
    plot(t_plot, vref, 'Color', colors.cmd, 'LineWidth', 2.2); grid on;
    ylabel('v_{ref} [V]');
    title(['Entrada: ' entry ' (promedio, con delay 0.5 s)']);
    ylim([-0.5, Vbus+0.5]);
    xline(T_delay,'k--','Delay','LabelVerticalAlignment','bottom');

    % (2) PWM digital 0/Vbus
    subplot(5,1,2);
    stairs(t_plot, vpwm_d, 'Color', colors.pwm, 'LineWidth', 1.0); grid on;
    ylabel('PWM [V] (digital)');
    title(sprintf('PWM conmutado (f_{pwm}=%g Hz) — ODE usa: %s', f_pwm, ternary(use_switching_in_ode,'conmutado','promedio')));
    ylim([-0.5, Vbus+0.5]);
    xline(T_delay,'k--');

    % (3) Vpwm_avg y v_emf
    subplot(5,1,3);
    plot(t_plot, vpwm_a, 'Color', colors.avg, 'LineWidth', 2.0); hold on;
    plot(t,      vemf,   'Color', colors.emf, 'LineWidth', 1.8); grid on;
    ylabel('[V]'); title('V_{pwm,avg}(t) y v_{emf}(t)');
    legend({'V_{pwm,avg}','v_{emf}'},'Location','best');
    ylim([-0.5, Vbus+0.5]);
    xline(T_delay,'k--');

    % (4) Velocidad angular (rad/s)
    subplot(5,1,4);
    plot(t, omegaa, 'Color', colors.w, 'LineWidth', 2.0); grid on;
    ylabel('\omega_a [rad/s]');
    title('Velocidad angular del brazo');

    % (5) Ángulo (rad) con línea de referencia en pi/2 y marca del evento
    subplot(5,1,5);
    plot(t, theta, 'Color', colors.th, 'LineWidth', 2.0); grid on; hold on;
    ylabel('\theta_a [rad]'); xlabel('Tiempo [s]');
    title('Ángulo del brazo (rad)');
    yline(theta_target,'k--','\pi/2','LineWidth',1.8,'LabelHorizontalAlignment','left');
    if ~isempty(te)
        xline(te(1),'r--','t_{hit}','LineWidth',1.2,'LabelVerticalAlignment','bottom');
    end

    sgtitle('Brazo con propulsor — Modelo NO lineal (delay 0.5 s, 0 \rightarrow \pi/2)','FontWeight','bold');
end

%% =================== FUNCIONES LOCALES ===================

function r = make_shape_with_delay(type, Tdelay, Tup)
% r(t) = 0 para t<Tdelay; en [Tdelay, Tdelay+Tup] sube 0->1 con la forma pedida;
% para t>Tdelay+Tup mantiene 1 (por si no se alcanza el objetivo antes).
    switch lower(type)
        case 'paso'
            r_seg = @(tau) (tau>=0);                        % salto
        case 'rampa'
            r_seg = @(tau) max(0,min(1, tau));              % 0->1 lineal
        case 'parabola'
            r_seg = @(tau) max(0,min(1, tau.^2));           % 0->1 cuadrática
        case 'seno'
            r_seg = @(tau) 0.5*(1 - cos(pi*max(0,min(1,tau)))); % raised-cosine
        otherwise
            error('Tipo de entrada no reconocido.');
    end
    r = @(t) (t<Tdelay).*0 + ...
             (t>=Tdelay & t<=Tdelay+Tup).*r_seg((t - Tdelay)/Tup) + ...
             (t>Tdelay+Tup).*1;
end

function duty_high = calibrate_duty_high(simHitFcn, d_min, d_max, theta_target)
% Búsqueda binaria de duty_high para alcanzar ~theta_target (rad) durante la subida.
    lo = max(0, d_min); hi = min(0.95, d_max);
    t_hi = simHitFcn(hi);
    if isnan(t_hi)    % ni con máx llega
        duty_high = hi; return;
    end
    for it=1:24
        mid  = 0.5*(lo+hi);
        tmid = simHitFcn(mid);
        if isnan(tmid)   % no alcanza -> subir duty
            lo = mid;
        else             % alcanza -> intenta bajar duty
            hi = mid;
        end
    end
    duty_high = 0.5*(lo+hi);
end

function t_hit = simulate_hit_time(duty_high, duty_low, r_base, Vbus, f_pwm, use_sw, p, T_total, theta_target)
% Simula y devuelve el tiempo donde theta cruza 'theta_target' (NaN si no cruza).
    [~, ~, vpwm_avg_fun, vpwm_sw_fun] = make_input_from_shape(r_base, duty_low, duty_high, Vbus, f_pwm);
    u_fun = vpwm_avg_fun; if use_sw, u_fun = vpwm_sw_fun; end
    x0 = [0;0;0];
    opts = odeset('RelTol',1e-7,'AbsTol',1e-9,'Events',@(t,x) event_theta_target(t,x,theta_target));
    try
        [~,~,te] = ode45(@(t,x) dyn_prop_arm_NL(t,x,u_fun,p), [0 T_total], x0, opts);
        if isempty(te), t_hit = NaN; else, t_hit = te(1); end
    catch
        t_hit = NaN;
    end
end

function [u_ref, duty_fun, vpwm_avg_fun, vpwm_sw_fun] = make_input_from_shape(r_base, duty_low, duty_high, Vbus, f_pwm)
% duty(t) entre duty_low y duty_high; v_ref = Vbus*duty; PWM digital con portadora triangular.
    clip01  = @(x) min(1,max(0,x));
    duty_fun     = @(t) clip01(duty_low + (duty_high - duty_low).*max(0,min(1,r_base(t))));
    u_ref        = @(t) Vbus .* duty_fun(t);          % promedio = entrada al modelo
    vpwm_avg_fun = u_ref;
    % PWM digital:
    T_pwm  = 1/f_pwm;
    tri01  = @(t) 2*abs(mod(t,T_pwm)/T_pwm - 0.5);    % 0..1
    vpwm_sw_fun  = @(t) Vbus .* double(duty_fun(t) > tri01(t));
end

function dx = dyn_prop_arm_NL(t,x,u_fun,p)
% Dinámica NO lineal (sin inductancia Lm): estados [theta; omega_a; v_emf]
    theta  = x(1); omegaa = x(2); vemf = x(3);
    vpwm   = u_fun(t);

    % Eléctrica (promediada, sin Lm)
    alpha = (p.Km*p.Ke)/(p.Rs+p.Rm);
    dvemf = ( alpha*(vpwm - vemf) - p.Kf*vemf ) / p.Jm;

    % Empuje cuadrático
    fm = p.kT_over_Ke2 * (vemf.^2);  % [N]

    % Dinámica del brazo (torques)
    tau = p.La*fm ...
        - p.Mm*p.g*p.La*cos(theta) ...
        - p.b_arm*omegaa ...
        - p.c_aero*omegaa.*abs(omegaa);

    dx = [omegaa;
          (1/p.Ja)*tau;
          dvemf];
end

function [value,isterminal,direction] = event_theta_target(~,x,theta_target)
    value = x(1) - theta_target; isterminal = 1; direction = +1;
end

function out = ternary(cond,a,b), if cond, out=a; else, out=b; end
end
