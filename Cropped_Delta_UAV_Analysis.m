%% ========================================================================
%  BSERC DEF-SPACE AUTUMN INTERNSHIP 2026 - AIRCRAFT DESIGN PROJECT 4
%  Title: Trim, Thrust Required, and Power Required Analysis of a
%         Cropped Delta Wing UAV Using MATLAB / Excel
%  Configuration: Tailless Cropped Delta Reflex Wing UAV (Wing-Alone)
%  Mentor: Dr. J. Sarathkumar Sebastin (VIT Bhopal University)
%  ========================================================================
clear; clc; close all;

fprintf('========================================================================\n');
fprintf('  CROPPED DELTA WING UAV - TRIM & PERFORMANCE SOLVER (MATLAB SUITE)\n');
fprintf('========================================================================\n\n');

%% 1. GIVEN GEOMETRIC, AERODYNAMIC, AND FLIGHT PARAMETERS
% --- Wing Planform Geometry (Ref: Project 4 Assignment Document) ---
Cr = 0.90;              % Root chord [m]
Ct = 0.15;              % Tip chord [m]
b  = 1.50;              % Wing span [m]
e  = 0.89;              % Oswald span efficiency factor [-]

% --- Aerodynamic & Longitudinal Stability Coefficients ---
% Ref: Project 4 Assignment & NASA TM-85767 / IIT Kanpur Delta Wing Dataset
CD0   = 0.03;           % Zero-lift parasite drag coefficient [-]
CL0   = 0.00;           % Zero-AoA lift coefficient (symmetric/reflex baseline) [-]
Cm0   = 0.01;           % Zero-lift pitching moment coefficient (reflex camber) [-]
CLa   = 2.92;           % Lift curve slope dCL/d(alpha) [per rad]
CLde  = 0.265;          % Elevon lift effectiveness dCL/d(delta_e) [per rad]
Cma   = -0.292;         % Longitudinal static stability derivative dCm/d(alpha) [per rad]
Cmde  = -0.40;          % Elevon pitch control derivative dCm/d(delta_e) [per rad]

% --- Flight & Atmospheric Conditions (Sea Level ISA) ---
m   = 3.50;             % UAV takeoff mass [kg]
g   = 10.0;             % Gravitational acceleration (as specified in assignment) [m/s^2]
rho = 1.225;            % Sea-level air density (ICAO ISA) [kg/m^3]
mu  = 1.81e-5;          % Sea-level dynamic viscosity (ICAO ISA / BSERC Table 14.1) [Pa.s]
a_s = 340.0;            % Sea-level speed of sound (ICAO ISA / BSERC Table 14.1) [m/s]

%% 2. PREPROCESSING GEOMETRIC & STABILITY CALCULATIONS
lambda = Ct / Cr;                                       % Taper ratio [-]
S      = (b / 2) * Cr * (1 + lambda);                   % Planform wing area [m^2]
AR     = (b^2) / S;                                     % Wing aspect ratio [-]
k      = 1 / (pi * e * AR);                             % Induced drag factor [-]
W      = m * g;                                         % UAV weight [N]
WS     = W / S;                                         % Wing loading [N/m^2]

% Mean Aerodynamic Chord (MAC) & Longitudinal Static Margin (Ref: BSERC Sec 10.4 & 15.4.1)
MAC    = (2/3) * Cr * ((1 + lambda + lambda^2) / (1 + lambda)); % MAC [m]
y_MAC  = (b / 6) * ((1 + 2*lambda) / (1 + lambda));             % Spanwise MAC station [m]
SM     = -Cma / CLa;                                            % Static Margin (fraction of MAC)

% Theoretical Optimal Benchmarks (Ref: BSERC Sec 14.4.1 & Anderson)
CL_LDmax      = sqrt(CD0 / k);                          % CL for Max (L/D) / Min Thrust
LD_max_theory = 1 / (2 * sqrt(CD0 * k));                % Theoretical Max (L/D)
Tr_min_theory = W / LD_max_theory;                      % Theoretical Minimum Thrust [N]
CL_Pmin       = sqrt(3 * CD0 / k);                      % CL for Min Power / Max Endurance
Endur_max_th  = (CL_Pmin^1.5) / (4 * CD0);              % Theoretical Max (CL^1.5 / CD)
alpha_de0_deg = rad2deg(-Cm0 / Cma);                    % AoA for zero elevator deflection [deg]

fprintf('--- PREPROCESSING GEOMETRY & STABILITY SUMMARY ---\n');
fprintf('  Taper Ratio (lambda)          : %.4f (1/6)\n', lambda);
fprintf('  Planform Area (S)             : %.4f m^2\n', S);
fprintf('  Aspect Ratio (AR)             : %.4f\n', AR);
fprintf('  Induced Drag Factor (k)       : %.4f\n', k);
fprintf('  Mean Aerodynamic Chord (MAC)  : %.4f m (at y = %.4f m)\n', MAC, y_MAC);
fprintf('  Static Margin (SM)            : %.4f (%.1f%% MAC ahead of NP)\n', SM, SM*100);
fprintf('  Aircraft Weight (W)           : %.2f N (Wing Loading = %.2f N/m^2)\n', W, WS);
fprintf('  Zero-Elevon Trim AoA          : %.3f deg\n', alpha_de0_deg);
fprintf('  Theoretical (L/D)_max         : %.3f at CL = %.4f (Tr_min = %.2f N)\n\n', ...
        LD_max_theory, CL_LDmax, Tr_min_theory);

%% 3. PHASE 1 & 2: ITERATIVE TRIM & LEVEL-FLIGHT PERFORMANCE SOLVER
alpha_deg = (0 : 0.5 : 12.0)';          % Angle of attack vector [deg]
N         = length(alpha_deg);          % Number of discrete flight states (25)

% Preallocate arrays
alpha_rad = zeros(N, 1);
de_rad    = zeros(N, 1);
de_deg    = zeros(N, 1);
CL_trim   = zeros(N, 1);
CD_ind    = zeros(N, 1);
CD_tot    = zeros(N, 1);
V_flight  = zeros(N, 1);
q_dyn     = zeros(N, 1);
Re_MAC    = zeros(N, 1);
Mach      = zeros(N, 1);
Tr_req    = zeros(N, 1);
Pr_req    = zeros(N, 1);
CL_CD     = zeros(N, 1);
CL15_CD   = zeros(N, 1);

% Electric Propulsion & Battery Sizing Parameters (Ref: BSERC Sec 14.9.2 & 16.7)
eta_prop  = 0.72;                       % Propeller efficiency (UIUC APC thin electric) [-]
eta_elec  = 0.88;                       % Brushless motor + ESC electrical efficiency [-]
P_avionics = 8.0;                       % Autopilot + receiver + servo power draw [W]
V_batt    = 14.8;                       % 4S Li-Po nominal voltage [V]
Q_Ah      = 5.0;                        % Battery capacity [Ah] (5000 mAh)
f_usable  = 0.80;                       % Safe depth-of-discharge usable fraction [-]
E_usable  = f_usable * V_batt * Q_Ah;   % Usable battery energy [Wh] (59.2 Wh)

P_shaft   = zeros(N, 1);
P_batt    = zeros(N, 1);
Endur_min = zeros(N, 1);
Range_km  = zeros(N, 1);

for i = 1:N
    % Step 1: Convert Angle of Attack to radians
    alpha_rad(i) = deg2rad(alpha_deg(i));
    
    % Step 2: Pitching Moment Equilibrium (Cm = Cm0 + Cma*alpha + Cmde*de = 0)
    de_rad(i) = -(Cm0 + Cma * alpha_rad(i)) / Cmde;
    de_deg(i) = rad2deg(de_rad(i));
    
    % Step 3: Trimmed Lift Coefficient
    CL_trim(i) = CL0 + CLa * alpha_rad(i) + CLde * de_rad(i);
    
    % Step 4: Steady Level Flight Velocity (L = W = 0.5 * rho * V^2 * S * CL)
    V_flight(i) = sqrt((2 * W) / (rho * S * CL_trim(i)));
    q_dyn(i)    = 0.5 * rho * V_flight(i)^2;
    
    % Step 5: Reynolds Number (based on MAC) and Mach Number
    Re_MAC(i) = (rho * V_flight(i) * MAC) / mu;
    Mach(i)   = V_flight(i) / a_s;
    
    % Step 6: Parabolic Drag Polar (CD = CD0 + k * CL^2)
    CD_ind(i) = k * (CL_trim(i)^2);
    CD_tot(i) = CD0 + CD_ind(i);
    
    % Step 7: Thrust Required (Tr = D) and Aerodynamic Power Required (Pr = Tr * V)
    Tr_req(i) = q_dyn(i) * S * CD_tot(i);
    Pr_req(i) = Tr_req(i) * V_flight(i);
    
    % Step 8: Aerodynamic Performance Metrics
    CL_CD(i)   = CL_trim(i) / CD_tot(i);
    CL15_CD(i) = (CL_trim(i)^1.5) / CD_tot(i);
    
    % Step 9: Advanced Extension - Shaft Power, Battery Endurance & Range
    P_shaft(i)   = Pr_req(i) / eta_prop;
    P_batt(i)    = (P_shaft(i) / eta_elec) + P_avionics;
    Endur_hr     = E_usable / P_batt(i);
    Endur_min(i) = Endur_hr * 60;
    Range_km(i)  = (V_flight(i) * Endur_hr * 3600) / 1000;
end

%% 4. DISPLAY COMPLETE RESULTS TABLE IN COMMAND WINDOW
ResultsTable = table(alpha_deg, de_rad, de_deg, CL_trim, V_flight, CD_tot, ...
                     Tr_req, Pr_req, CL_CD, CL15_CD, Endur_min, Range_km, ...
    'VariableNames', {'Alpha_deg', 'DeltaE_rad', 'DeltaE_deg', 'CL_trim', ...
                      'Velocity_ms', 'CD_total', 'Thrust_N', 'Power_W', ...
                      'CL_by_CD', 'CL15_by_CD', 'Endurance_min', 'Range_km'});
disp(ResultsTable);

% Export table to CSV
writetable(ResultsTable, 'Cropped_Delta_UAV_Trim_Table.csv');

%% 5. PHASE 3: VISUALIZATION (MANDATORY + ADVANCED PLOTS)
if ~exist('Plots', 'dir')
    mkdir('Plots');
end

% Plot 1: Trim Elevator Deflection vs Angle of Attack
figure('Color', 'w', 'Position', [100, 100, 800, 500]);
plot(alpha_deg, de_deg, '-ob', 'LineWidth', 2, 'MarkerFaceColor', 'b');
hold on; yline(0, '--k', 'Zero Deflection (\delta_e = 0^\circ)');
plot(alpha_de0_deg, 0, 'pr', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; xlabel('Angle of Attack, \alpha (deg)');
ylabel('Trim Elevon Deflection, \delta_{e,trim} (deg)');
title('Trim Elevon Deflection vs. Angle of Attack');
saveas(gcf, 'Plots/MATLAB_Plot1_Trim_Elevator_vs_AoA.png');

% Plot 2: Thrust Required vs Flight Velocity
figure('Color', 'w', 'Position', [100, 100, 800, 500]);
plot(V_flight(2:end), Tr_req(2:end), '-sr', 'LineWidth', 2, 'MarkerFaceColor', 'r');
grid on; xlabel('Flight Velocity, V (m/s)');
ylabel('Thrust Required, T_r (N)');
title('Thrust Required vs. Flight Velocity (Steady Level Flight)');
saveas(gcf, 'Plots/MATLAB_Plot2_Thrust_vs_Velocity.png');

% Plot 3: Power Required vs Flight Velocity
figure('Color', 'w', 'Position', [100, 100, 800, 500]);
plot(V_flight(2:end), Pr_req(2:end), '-^m', 'LineWidth', 2, 'MarkerFaceColor', 'm');
grid on; xlabel('Flight Velocity, V (m/s)');
ylabel('Aerodynamic Power Required, P_r (W)');
title('Power Required vs. Flight Velocity (Steady Level Flight)');
saveas(gcf, 'Plots/MATLAB_Plot3_Power_vs_Velocity.png');

% Plot 4: Aerodynamic Efficiency Metrics (CL/CD and CL^(3/2)/CD) vs AoA
figure('Color', 'w', 'Position', [100, 100, 800, 500]);
yyaxis left;
plot(alpha_deg, CL_CD, '-ob', 'LineWidth', 2, 'MarkerFaceColor', 'b');
ylabel('Lift-to-Drag Ratio, C_L / C_D (-)');
yyaxis right;
plot(alpha_deg, CL15_CD, '-sg', 'LineWidth', 2, 'MarkerFaceColor', 'g');
ylabel('Endurance Parameter, C_L^{3/2} / C_D (-)');
grid on; xlabel('Angle of Attack, \alpha (deg)');
title('Aerodynamic Efficiency Metrics vs. Angle of Attack');
saveas(gcf, 'Plots/MATLAB_Plot4_Efficiency_vs_AoA.png');

fprintf('\nSimulation complete! CSV table and plots exported successfully.\n');
