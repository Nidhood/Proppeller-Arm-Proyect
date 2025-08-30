% Script: pole_zero_prop_arm.m
% Plot pole–zero map with labels (poles = red x, zeros = blue o)

close all; clear; clc;

% ----------------- Parameters -----------------
Ke = 5.5e-3;   % back-emf per rad/s
Km = 5.5e-3;   % torque constant [N·m/A]
Jm = 3e-6;     % motor inertia [kg·m^2]
La1 = 0.21;    % arm length (one) [m]
La2 = 0.21;    % arm length (two) [m]
Ja  = 1/3*(0.014)*La1^3 + 0.016*La1^2 + 1/3*(0.014)*La2^3 + 0.016*La2^2;
Rm = 1;        % motor resistance [ohm]
Rs = 1;        % series resistance [ohm]
Kf = 10e-6;    % viscous friction [N·m·s]
Kt = 1.8e-3;   % arm torque coefficient

% Choose effective arm length for the numerator if needed
La = La1;

% ----------------- Transfer function G(s) = Y(s)/U(s) -----------------
a   = (1/Jm) * (Km*Ke/(Rs+Rm) + Kf);           % mechanical pole term
num = (Kt*Km*La) / (Ja*Jm*(Rs+Rm));            % constant numerator
den = [1, a, 0, 0];                            % s^3 + a s^2

G = tf(num, den);

% ----------------- Poles and zeros -----------------
p = pole(G);
z = zero(G);    % empty for this model (strictly proper)

% ----------------- Plot -----------------
figure('Color','w'); hold on; grid on; box on;
title('Pole–Zero Map of Prop-Arm Transfer Function');
xlabel('Re\{s\}'); ylabel('Im\{s\}');

% Axis range based on poles/zeros
allrz = [p; z];
if isempty(allrz), allrz = [ -1; -a ]; end
xr = [min(real(allrz)) max(real(allrz))];
yr = [min(imag(allrz)) max(imag(allrz))];
padx = 0.15*max(1, diff(xr)); pady = 0.3*max(1, diff(yr)+1e-3);
xlim([xr(1)-padx, xr(2)+padx]); ylim([yr(1)-pady, yr(2)+pady]);
plot(xlim, [0 0], 'k:'); plot([0 0], ylim, 'k:');

% Plot zeros (blue circles)
if ~isempty(z)
    scatter(real(z), imag(z), 90, 'bo', 'filled', 'DisplayName','Zeros');
else
    plot(nan, nan, 'bo', 'DisplayName','Zeros'); % keep legend
end

% Plot poles (red crosses)
scatter(real(p), imag(p), 90, 'rx', 'LineWidth', 2, 'DisplayName','Poles');
legend('Location','best');

% ----------------- Label roots with values -----------------
tol = 1e-9;       % grouping tolerance
label_roots(z, 'b', tol);
label_roots(p, 'r', tol);

% Also show MATLAB built-in view
figure('Color','w'); pzmap(G); grid on; title('pzmap(G)');

% =================== Local function ===================
function label_roots(vals, color, tol)
    % Group (nearly) identical roots and place a text label above each.
    if isempty(vals), return; end
    used = false(numel(vals),1);
    for k = 1:numel(vals)
        if used(k), continue; end
        same = abs(real(vals)-real(vals(k)))<tol & abs(imag(vals)-imag(vals(k)))<tol;
        group = vals(same);
        used(same) = true;

        w = mean(group);
        % Build text "a+bi" or just "a" if imaginary part ~ 0
        if abs(imag(w)) < 1e-9
            txt = sprintf('%.4g', real(w));
        else
            txt = sprintf('%.4g%+.4gi', real(w), imag(w));
        end
        if numel(group) > 1
            txt = sprintf('%s (\\times%d)', txt, numel(group));
        end

        % Small vertical offset for readability
        offs = 0.02 * (1 + 0.1*numel(group));
        text(real(w), imag(w)+offs, txt, 'Color', color, ...
             'FontSize', 11, 'HorizontalAlignment','center', ...
             'FontWeight', 'bold');
    end
end
