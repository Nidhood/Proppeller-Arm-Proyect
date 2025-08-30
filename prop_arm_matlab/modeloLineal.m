% ========== arm_propeller_LINEAR_0p5s_delay_to_90deg_OFFSET_10V.m ==========
% Modelo lineal (variables incrementales) alrededor del equilibrio (hover).
% Entradas: Paso / Rampa / Parabola / Seno.
% CAMBIO CLAVE:
%   - La referencia v_ref arranca en v_pwm0 y, tras el delay, solo sube
%     hasta 10 V con la forma elegida: v_ref = v_pwm0 + (10 - v_pwm0)*r_base(t).
%   - La simulación usa u = v_ref - v_pwm0 (incremental). Antes del delay: u=0.
%   - PWM digital parte en duty_eq y respeta saturación [0,1].

close all; clear; clc;

%% ---------------- Parámetros físicos ----------------
p = struct();
p.Ke = 5.5e-3;          % [V/(rad/s)]
p.Km = 5.5e-3;          % [N*m/A]
p.Jm = 3e-6;            % [kg*m^2]
p.La1 = 0.21; p.La2 = 0.21;  % [m]
p.Rm = 1;  p.Rs = 1;          % [ohm]
p.Kf = 10e-6;                  % [N*m*s]
p.g  = 9.81;                   % [m/s^2]
p.Mm = 0.06;                   % [kg]

% Inercia del brazo
p.m1=0.014; p.m2=0.014; p.mh1=0.016; p.mh2=0.016;
p.La = p.La1;
p.Ja = (1/3)*p.m1*p.La1^2 + p.mh1*p.La1^2 + (1/3)*p.m2*p.La2^2 + p.mh2*p.La2^2;

% Rozamiento brazo
p.b_arm = 5e-3;               % [N*m*s]

% Signo geométrico del torque (ajusta si tu convención es opuesta)
p.sgn_tau_mech = +1;

% Empuje cuadrático: f_m = kT*(omega_m)^2,  omega_m = v_emf/Ke
omega_ref     = 300;            % [rad/s]
Kt_arm_spec   = 1.8e-3;         % [N*s/rad]  (2*kT*omega_ref = Kt_arm_spec)
kT            = Kt_arm_spec/(2*omega_ref);     % [N*s^2/rad^2]
p.kT_over_Ke2 = kT/p.Ke^2;      % [N/V^2]

% Fuente y PWM
Vbus    = 11;        % [V] (bus disponible)
f_pwm   = 500;       % [Hz] solo para visualización

%% --------- Equilibrio (theta0 = 0, hover) ----------
alpha_fm = p.kT_over_Ke2;                            % [N/V^2]
v_emf0   = sqrt((p.Mm*p.g)/alpha_fm);               % [V]
v_pwm0   = v_emf0*( 1 + p.Kf*(p.Rs+p.Rm)/(p.Km*p.Ke) );
duty_eq  = min(max(v_pwm0/Vbus,0),1);
omega_m0 = v_emf0/p.Ke;

fprintf('Equilibrio: v_emf0=%.3f V, v_pwm0=%.3f V, duty_eq=%.3f\n',...
        v_emf0, v_pwm0, duty_eq);

%% --------- Sistema lineal incremental ---------
% x = [dtheta; domega_a; d v_emf] ; u = d v_pwm ; y = dtheta
alpha_e   = (p.Km*p.Ke)/(p.Rs+p.Rm);                 % [V*s/rad]
dFm_dVemf = 2 * p.kT_over_Ke2 * v_emf0;              % [N/V] pendiente en el eq.

A22 = -p.b_arm/p.Ja;
A23 = p.sgn_tau_mech * (p.La/p.Ja) * dFm_dVemf;
A33 = -(alpha_e + p.Kf)/p.Jm;
B3  =  alpha_e/p.Jm;

A = [0   1   0  ;
     0  A22  A23;
     0   0   A33];
B = [0; 0; B3];
C = [1 0 0];  D = 0;

sys_lin = ss(A,B,C,D);

%% ---------------- Tiempo y delay ----------------
T_delay = 0.5;     % [s]
T_up    = 1.2;     % [s] duración de la subida
T_total = T_delay + T_up + 1.0;

% Límite superior deseado para la referencia promedio
Vcap    = 10.0;                             % [V] *** tu tope deseado ***
Vspan   = max(0, Vcap - v_pwm0);            % lo que falta desde el equilibrio hasta 10 V

% Ángulo objetivo y evento
theta_target = pi/2;

%% ---------------- Entradas y paleta ----------------
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

%% ---------------- Bucle por cada entrada ----------------
for k = 1:numel(types)
    entry  = types{k};
    colors = pal.(entry);

    % r_base(t) en [0,1] con delay y subida
    r_base = make_shape_with_delay(entry, T_delay, T_up);

    % ====== Construcción de la ENTRADA ======
    % Promedio visible: arranca en v_pwm0 y sube como la forma hasta 10 V
    vref_fun = @(t) v_pwm0 + Vspan .* r_base(t);

    % PWM digital (duty sobre Vbus; parte en duty_eq)
    T_pwm  = 1/f_pwm;
    tri01  = @(t) 2*abs(mod(t,T_pwm)/T_pwm - 0.5);        % 0..1
    duty_fun = @(t) min(1,max(0, vref_fun(t)/Vbus));
    vpwm_sw_fun = @(t) Vbus .* double(duty_fun(t) > tri01(t));

    % ====== Simulación LINEAL incremental ======
    t_sim = linspace(0, T_total, 8000);
    vref  = vref_fun(t_sim);
    u     = vref - v_pwm0;        % entrada incremental (u=0 antes del delay)

    [theta_delta, ~, X_delta] = lsim(sys_lin, u, t_sim, [0;0;0]);

    theta   = theta_delta;        % theta0=0
    omega_a = X_delta(:,2);
    v_emf   = v_emf0 + X_delta(:,3);

    % Evento de parada (si llega)
    hit_idx = find(theta >= theta_target, 1, 'first');
    if ~isempty(hit_idx)
        te = t_sim(hit_idx);
        t_sim  = t_sim(1:hit_idx);
        theta  = theta(1:hit_idx);
        omega_a = omega_a(1:hit_idx);
        v_emf  = v_emf(1:hit_idx);
        vref   = vref(1:hit_idx);
    else
        te = [];
    end

    % Señales densas para gráficos
    t_plot = linspace(0, max(max(t_sim), T_total), 4000);
    vref_plot = v_pwm0 + Vspan .* r_base(t_plot);
    vpwm_d    = vpwm_sw_fun(t_plot);

    %% --------- FIGURA (5 subplots) ---------
    figure('Name',['LINEAR-offset10V - ' entry], ...
           'Position',[80 60 1200 920],'Color','w');

    % (1) v_ref(t) (promedio)
    subplot(5,1,1);
    plot(t_plot, vref_plot, 'Color', colors.cmd, 'LineWidth', 2.2); grid on;
    ylabel('v_{ref} [V]');
    title(['LINEAR - Input: ' entry ' (offset en v_{pwm0}, tope 10 V)']);
    ylim([-0.5, max(10.5,Vbus)+0.5]);
    xline(T_delay,'k--','Delay','LabelVerticalAlignment','bottom');

    % (2) PWM digital 0/Vbus
    subplot(5,1,2);
    stairs(t_plot, vpwm_d, 'Color', colors.pwm, 'LineWidth', 1.0); grid on;
    ylabel('PWM [V] (digital)');
    title(sprintf('Switched PWM (f_{pwm}=%g Hz)', f_pwm));
    ylim([-0.5, Vbus+0.5]);
    xline(T_delay,'k--');

    % (3) Vpwm_avg y v_emf
    subplot(5,1,3);
    plot(t_plot, vref_plot, 'Color', colors.avg, 'LineWidth', 2.0); hold on;
    plot(t_sim, v_emf, 'Color', colors.emf, 'LineWidth', 1.8); grid on;
    ylabel('[V]'); title('V_{pwm,avg}(t) y v_{emf}(t)');
    legend({'V_{pwm,avg}','v_{emf}'},'Location','best');
    ylim([-0.5, max(10.5,Vbus)+0.5]);
    xline(T_delay,'k--');

    % (4) Velocidad angular
    subplot(5,1,4);
    plot(t_sim, omega_a, 'Color', colors.w, 'LineWidth', 2.0); grid on;
    ylabel('\omega_a [rad/s]');
    title('Arm angular velocity (LINEAR)');

    % (5) Ángulo
    subplot(5,1,5);
    plot(t_sim, theta, 'Color', colors.th, 'LineWidth', 2.0); grid on; hold on;
    ylabel('\theta_a [rad]'); xlabel('Tiempo [s]');
    title('Arm angle (rad) - LINEAR MODEL');
    yline(theta_target,'k--','\pi/2','LineWidth',1.8,'LabelHorizontalAlignment','left');
    if ~isempty(te)
        xline(te,'r--','t_{hit}','LineWidth',1.2,'LabelVerticalAlignment','bottom');
    end

    sgtitle('Propeller arm — LINEAR (offset en equilibrio, tope 10 V)','FontWeight','bold');
end

%% =================== FUNCIONES LOCALES ===================
function r = make_shape_with_delay(type, Tdelay, Tup)
    switch lower(type)
        case 'paso'
            r_seg = @(tau) (tau>=0);
        case 'rampa'
            r_seg = @(tau) max(0,min(1, tau));
        case 'parabola'
            r_seg = @(tau) max(0,min(1, tau.^2));
        case 'seno'
            r_seg = @(tau) 0.5*(1 - cos(pi*max(0,min(1,tau))));
        otherwise
            error('Tipo de entrada no reconocido.');
    end
    r = @(t) (t<Tdelay).*0 + ...
             (t>=Tdelay & t<=Tdelay+Tup).*r_seg((t - Tdelay)/Tup) + ...
             (t>Tdelay+Tup).*1;
end
