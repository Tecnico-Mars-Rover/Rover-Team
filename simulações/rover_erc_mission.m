clc;
clear;
close all;

%% =========================================================================
%  SIMULAÇÃO DE NAVEGAÇÃO ERC — ROVER 75 kg  (Navigation Task)
%  Versão: Distance-Based Terrain + Slip Limit + Stop/Go com Inércia
%
%  Arquitetura do modelo:
%    Terreno:    Perfil de altitude ancorado à POSIÇÃO (não ao tempo).
%                Garante que a altitude é geograficamente consistente
%                independentemente da velocidade do rover — elimina o
%                "altitude drift" das versões baseadas em tempo.
%    Física:     Força de tração limitada por coeficiente de Coulomb do
%                solo (mu_soil). O rover não consegue subir se a força
%                necessária exceder F_slip_limit.
%    Controlo:   Gerador de rampa de velocidade (v_ref) com limites de
%                aceleração e desaceleração físicos. Stop/go realista.
%    Energia:    Modelo de Thévenin: R_int = resistência interna do pack.
%                Produz voltage sag (queda de tensão) sob corrente elevada.
%
%  Mars Yard ERC — Perfil de terreno baseado em Losiak et al. (2023):
%    Área física: ~35x45 m. Percurso total odometria: ~1200 m (loops).
%    Features: dunes, crater A (prof 2m), inselberg (2.2m), cratera B,
%              vulcão (pico 2.5m acima base), delta aluvial, outcrops,
%              cratera C, return path. Altitude final = 0 m (loop fechado).
%
%  Referências:
%    - Losiak et al. (2023) "Mars Yard Design During ERC 2020-2022"
%    - ERC Rulebook: v_max=1 m/s, slope ERC nav ~20-25 deg
%    - Análise de Eficiências - Equipa Rover IST N3E (2026)
%    - Projeto VIENA, IST 2018-2019
% =========================================================================

%% =========================================================================
%  1) PARÂMETROS — ALTERAR AQUI
% =========================================================================

% --- Veículo ---
M             = 75;     % Massa total do rover                    [kg]
fm            = 1.10;   % Fator de massa rotacional               [-]
rw            = 0.125;  % Raio da roda                            [m]
n_mot         = 4;      % Número de motores (4WD)                 [-]
Tw_max        = 35;     % Binário máximo por motor                [Nm]

% --- Cinemática ---
v_target      = 1.0;    % Velocidade alvo (máx regulamento ERC)   [m/s]
a_accel       = 0.5;    % Aceleração máx da rampa de referência   [m/s^2]
a_decel       = 1.0;    % Desaceleração máx (travagem)            [m/s^2]
%  Nota: F_max/(fm*M) = 13.6 m/s^2 (máximo físico absoluto dos motores)
%  Usamos 0.5 m/s^2 para representar controlo suave e resposta real do rover

% --- Terreno ---
theta_max_deg = 45;     % <<< INCLINAÇÃO MÁXIMA DO PERCURSO [deg] >>>
                        %     Valores ERC típicos: 15 (fácil), 20 (médio), 25 (standard), 30 (severo)
                        %     Este parâmetro escala todo o mapa topográfico de forma proporcional.
alt_peak_max  = 10.0;   % <<< ALTURA MÁXIMA DO PICO [m] >>>
                        %     ERC real: ~2m (arena 35x45m). Stress Test: 5-15m.
                        %     Aumentar estica verticalmente o terreno (slopes conservados).

% --- Solo / Fricção ---
phi_soil_deg  = 35;     % Ângulo de fricção interna do rególito   [deg]
%  mu_soil = tan(35) = 0.70 (solo arenoso marciano - JEZ-1 analogue)
%  Limita a força de tração máxima transmissível ao solo (Coulomb)
mu_soil       = tan(deg2rad(phi_soil_deg));

% --- Coeficientes de Rolamento por Zona ---
Crr_sand      = 0.12;   % Areia granular (base)                   [-]
Crr_soft      = 0.18;   % Solo mole — delta aluvial               [-]
Crr_rock      = 0.10;   % Rocha firme — outcrops                  [-]

% --- Validação Física: theta_max_deg vs Limite de Coulomb do Solo ---
%  Para subir um declive theta, o rover precisa que:
%    F_trac > M*g*sin(theta) + Crr*M*g*cos(theta)
%  Mas F_trac <= mu_soil * M*g*cos(theta)  (Coulomb limit)
%  => mu_soil * cos(theta) > sin(theta) + Crr*cos(theta)
%  => theta < atan(mu_soil - Crr)   [aprox, ignorando aceleração]
theta_coulomb_max = rad2deg(atan(mu_soil - Crr_sand));  % ~30.6 deg
theta_safe_max    = theta_coulomb_max * 0.90;           % margem 10%
if theta_max_deg > theta_coulomb_max
    warning(['theta_max_deg=%.0f deg excede o limite de Coulomb do JEZ-1 ' ...
             '(phi=%.0f deg -> theta_max_fisico=%.1f deg). ' ...
             'O rover ficara bloqueado. Ajustado para %.1f deg (margem 10%%).'], ...
             theta_max_deg, phi_soil_deg, theta_coulomb_max, theta_safe_max);
    theta_max_deg = theta_safe_max;
    fprintf('  *** theta_max_deg ajustado para %.1f deg (limite Coulomb JEZ-1) ***\n\n', theta_max_deg);
end

% --- Sistema Energético ---
g             = 9.81;   % Gravidade                               [m/s^2]
eta_motor     = 0.85;   % Eficiência motor BLDC                   [-]
eta_gear      = 0.84;   % Eficiência caixa planetária             [-]
%  eta_total = eta_motor * eta_gear = 0.716 (nominal, velocidade normal)
%  Em stall (v<0.15 m/s e FT>50% F_max) cai para eta_stall (ver loop)
eta_stall     = 0.40;   % Eficiência em stall (alta corrente)     [-]
V_nom         = 48;     % Tensão nominal do pack                  [V]
R_int         = 0.10;   % Resistência interna do pack (ESR)       [Ohm]
%  Voltage sag: V_bat = V_nom - I_bat * R_int
%  A 20A: sag = 2V (48V -> 46V). Realista para pack LiPo 48V/20Ah.
P_electronics = 50;     % Consumo base eletrónica (excluindo tração) [W]
DOD_max       = 0.80;   % Profundidade de descarga máxima         [-]

% --- Simulação ---
dt            = 0.1;    % Passo de tempo                          [s]
t_total       = 3600;   % Duração total (60 min)                  [s]
rng_seed      = 42;     % Semente para reprodutibilidade          [-]

% --- Paragens (Stop/Go da missão) ---
n_stops_target = 14;    % <<< NÚMERO DE PARAGENS DESEJADO >>>
                        %     Cada paragem representa: observação científica,
                        %     desvio de obstáculo, E-Stop, sinalização (Req#15-16)
stop_dur_min  = 10;     % Duração mínima de cada paragem       [s]
stop_dur_max  = 45;     % Duração máxima de cada paragem       [s]

%% =========================================================================
%  2) VETOR DE TEMPO
% =========================================================================
t = 0 : dt : t_total;
N = length(t);

%% =========================================================================
%  3) MAPA TOPOGRÁFICO ANCORADO À DISTÂNCIA
%
%  O perfil de altitude é construído como soma de Gaussianas centradas em
%  posições geográficas fixas ao longo do percurso.
%
%  Cada feature tem:
%    d_c    — posição central ao longo do percurso         [m]
%    peak   — altura máxima/profundidade máxima            [m]
%    sigma  — calculado para produzir o slope desejado     [m]
%             sigma = |peak| / (tan(slope_deg) * sqrt(e))
%
%  A inclinação theta(pos) é derivada analiticamente de d(alt)/d(pos).
%  Como o terreno é função da posição e não do tempo, a altitude integrada
%  durante a simulação é geograficamente consistente: sobe e desce o mesmo
%  número de metros independentemente da velocidade do rover.
%
%  Mapa de Crr:  [0-450m]   Crr_sand
%                [450-700m] Crr_soft  (delta aluvial)
%                [700-900m] Crr_rock  (outcrops rochosos)
%                [900-1200m]Crr_sand  (return path)
%
%  Altitude esperada: max +2.5m (vulcão), min -2.0m (crater A)
%  Theta esperado: max ±24 deg (dentro do limite ERC de navegação)
%  Percurso total: 1200m (loops dentro do Mars Yard 35x45m)
% =========================================================================
dp    = 0.5;               % Resolução do mapa espacial           [m]
d_max = 4000;              % Mapa cobre distância máxima possível [m]
pos_map = 0 : dp : d_max; % Vetor de posições do mapa

% =========================================================================
%  MAPA TOPOGRÁFICO: 3 Loops (~1200m cada) sobre a arena de 35x45m
%
%  Cada loop é um circuito fechado (altitude final = altitude inicial).
%  Para garantir o fecho, a soma líquida de 'peak' por loop é ≈ 0m.
%
%  Loop 1: 0–1200m   (Egress → Crater A → Inselberg → Vulcão → Return I)
%  Loop 2: 1200–2400m (Dunes → Crater B → Outcrops → Return II)
%  Loop 3: 2400–3600m (Return III — percurso de regresso mais suave)
%
%  As Gaussianas usam sigma calculado para slope pretendido:
%    sigma = |peak| / (tan(slope_deg) * sqrt(e))
% =========================================================================

% Loop 1 (0–1200m) — features principais ERC
bumps_L1 = [
%   d_c    peak   sigma   % feature
     15,   -0.8,  3.45;   % egress ramp -8 deg
     55,   +0.6,  2.06;   % dune 1 +10 deg
     90,   -0.6,  2.06;   % hollow 1 (fecha dune 1)
    135,   -2.0,  3.00;   % CRATER A (-22 deg, prof 2m)
    200,   +2.0,  3.45;   % INSELBERG +20 deg (fecha crater A)
    270,   +0.5,  1.72;   % dune 2
    310,   -0.5,  1.72;   % hollow 2 (fecha dune 2)
    360,   -1.0,  1.87;   % crater B -18 deg
    420,   +1.0,  1.87;   % colina B +18 deg (fecha crater B)
    500,   +1.0,  2.85;   % abordagem vulcão
    545,   +1.5,  2.50;   % VULCÃO pico +20 deg
    595,   -1.5,  2.50;   % descida vulcão norte -20 deg (fecha vulcão)
    670,   -0.3,  2.08;   % delta aluvial
    750,   -1.4,  2.61;   % crater C -18 deg
    810,   +1.4,  2.61;   % saída crater C +18 deg (fecha)
    860,   +0.5,  1.72;   % outcrop 1
    900,   -0.5,  1.72;   % hollow 3 (fecha outcrop 1)
    950,   -0.8,  3.02;   % depressão return
   1020,   +0.8,  3.02;   % saída depressão (fecha)
   1080,   +0.3,  2.50;   % micro-lomba regresso
   1150,   -0.3,  2.50;   % micro-hollow (fecha, altitude ≈ 0)
];

% Loop 2 (1200–2400m) — variante com menos features dominantes
% --- Escalar picos para que theta_max_deg seja atingido no pior obstáculo ---
% Calcula o slope atual (antes do escalonamento) derivando analiticamente o mapa base de L1
dp_tmp   = 0.1;
pos_tmp  = 0 : dp_tmp : 1200;
alt_tmp  = zeros(1, length(pos_tmp));
for b = 1 : size(bumps_L1, 1)
    alt_tmp = alt_tmp + bumps_L1(b,2) * exp(-0.5*((pos_tmp - bumps_L1(b,1))/bumps_L1(b,3)).^2);
end
slope_current_deg = max(abs(rad2deg(atan(gradient(alt_tmp, dp_tmp)))));
if slope_current_deg > 0
    scale_factor = tan(deg2rad(theta_max_deg)) / tan(deg2rad(slope_current_deg));
else
    scale_factor = 1;
end
bumps_L1(:,2) = bumps_L1(:,2) * scale_factor;  % re-escalar picos para theta_max_deg

% --- Escalar altura absoluta para alt_peak_max ---
% Os slopes mantêm-se em theta_max_deg porque picos e sigmas escalam na mesma proporção.
current_peak_max = max(abs(bumps_L1(:, 2)));
if current_peak_max > 0
    alt_scale = alt_peak_max / current_peak_max;
    bumps_L1(:, 2) = bumps_L1(:, 2) * alt_scale;   % escala as alturas
    bumps_L1(:, 3) = bumps_L1(:, 3) * alt_scale;   % escala as larguras (slope conservado)
end

bumps_L2 = bumps_L1;
bumps_L2(:,1) = bumps_L1(:,1) + 1200;     % deslocar 1200m
bumps_L2(:,2) = bumps_L1(:,2) * 0.75;    % 25% mais suave (terreno explorado)

% Loop 3 (2400–3600m) — regresso, ainda mais suave
bumps_L3 = bumps_L1;
bumps_L3(:,1) = bumps_L1(:,1) + 2400;
bumps_L3(:,2) = bumps_L1(:,2) * 0.50;

terrain_bumps = [bumps_L1; bumps_L2; bumps_L3];
n_bumps       = size(terrain_bumps, 1);
alt_map       = zeros(1, length(pos_map));

for b = 1 : n_bumps
    d_c   = terrain_bumps(b, 1);
    peak  = terrain_bumps(b, 2);
    sigma = terrain_bumps(b, 3);
    alt_map = alt_map + peak * exp(-0.5 * ((pos_map - d_c) / sigma).^2);
end

% Derivar theta do gradiente de altitude (geometricamente exato)
dalt_dp       = gradient(alt_map, dp);
theta_map_deg = rad2deg(atan(dalt_dp));

% Suavização mínima (eliminar artefactos numéricos do gradiente)
theta_map_deg = movmean(theta_map_deg, 3);

% Micro-rugosidades do solo (trepidação real sobre pedras e irregularidades)
rng(rng_seed);
noise_amp     = 1.5;  % amplitude máxima de ruído: ±1.5 deg
noise_raw     = noise_amp * randn(1, length(pos_map));
theta_map_deg = theta_map_deg + movmean(noise_raw, 8);

% Crr mapa espacial — repete a cada 1200m (por loop)
Crr_map = Crr_sand * ones(1, length(pos_map));
for loop_off = [0, 1200, 2400]
    Crr_map(pos_map >= loop_off+450 & pos_map <= loop_off+700) = Crr_soft;
    Crr_map(pos_map >= loop_off+700 & pos_map <= loop_off+900) = Crr_rock;
end

%% =========================================================================
%  4) AGENDA DE PARAGENS — stop/go realista com inércia
%
%  Geradas com espaçamento e duração aleatórios (semente fixa).
%  Representam: obstáculos, reposicionamento, espera de comando,
%               E-Stop regulamentar, sinalização de aviso (ERC Req #15-16).
%
%  Parâmetros baseados no ERC: gaps 150-350s, duração 10-45s.
%  Sem paragens nos primeiros 2 min (egress) e últimos 3 min (return).
% =========================================================================
rng(rng_seed + 1);

% Calculação automática do espaçamento para atingir n_stops_target
t_mission_usable = t_total - 120 - 180; % janela util (sem egress nem return)
stop_gap_nom  = round(t_mission_usable / max(n_stops_target, 1));
stop_gap_min  = max(30, stop_gap_nom - 60);
stop_gap_max  = stop_gap_nom + 60;

stop_list = [];
t_probe   = 120;
while t_probe < 3300
    gap_s   = randi([stop_gap_min, stop_gap_max]);
    t_probe = t_probe + gap_s;
    if t_probe >= 3300; break; end
    dur_s = randi([stop_dur_min, stop_dur_max]);
    stop_list(end+1, :) = [t_probe, dur_s]; %#ok<AGROW>
end
n_stops = size(stop_list, 1);

is_stopped = false(1, N);
for s = 1 : n_stops
    is_stopped(t >= stop_list(s,1) & t <= stop_list(s,1)+stop_list(s,2)) = true;
end

%% =========================================================================
%  5) PRÉ-ALOCAÇÃO
% =========================================================================
v            = zeros(1, N);   % velocidade linear                 [m/s]
v_ref        = zeros(1, N);   % referência de velocidade (rampa)  [m/s]
pos          = zeros(1, N);   % posição odométrica acumulada      [m]
altitude     = zeros(1, N);   % altitude acumulada                [m]
FT_vec       = zeros(1, N);   % força de tração real (após slip)  [N]
Tw_vec       = zeros(1, N);   % binário por motor                 [Nm]
I_bat_vec    = zeros(1, N);   % corrente da bateria               [A]
V_bat_vec    = zeros(1, N);   % tensão da bateria                 [V]
P_elec_vec   = zeros(1, N);   % potência eléctrica total          [W]
E_Wh_vec     = zeros(1, N);   % energia acumulada                 [Wh]
Ah_vec       = zeros(1, N);   % carga extraída acumulada          [Ah]
theta_log    = zeros(1, N);   % log de inclinação                 [deg]
Crr_log      = zeros(1, N);   % log de Crr                        [-]

%% =========================================================================
%  6) LOOP DE SIMULAÇÃO — integração de Euler
%
%  A cada passo:
%  (a) Lookup do mapa espacial na posição atual do rover
%  (b) Forças resistivas: F_slope + F_roll
%  (c) Limite de slip: FT <= mu_soil * N_force (Coulomb)
%  (d) Feedforward de aceleração + correção proporcional
%  (e) Dinâmica de Euler: a = (FT - F_res)/(fm*M)
%  (f) Potência eléctrica com eficiência dinâmica + modelo Thévenin
% =========================================================================
F_max   = (Tw_max / rw) * n_mot;   % Força máxima motores: 1120 N
Kp_cor  = fm * M * 2.0;            % Ganho proporcional de correção

for i = 1 : N

    % (a) Lookup do mapa de terreno na posição atual
    idx_map = max(1, min(length(pos_map), floor(pos(i)/dp) + 1));
    th_deg  = theta_map_deg(idx_map);
    Crr     = Crr_map(idx_map);
    theta_log(i) = th_deg;
    Crr_log(i)   = Crr;
    th = deg2rad(th_deg);

    % (b) Forças resistivas
    F_slope = M * g * sin(th);
    F_roll  = M * g * abs(cos(th)) * Crr;
    F_res   = F_slope + F_roll;

    % (c) Limite de tração por slip (Coulomb)
    N_force      = M * g * abs(cos(th));
    F_slip_limit = N_force * mu_soil;   % máx transmissível ao solo

    % (d) Gerador de rampa de velocidade
    if is_stopped(i)
        dv_ref_dt = -a_decel;
    else
        dv_ref_dt = +a_accel;
    end
    if i < N
        v_ref(i+1) = max(0, min(v_target, v_ref(i) + dv_ref_dt * dt));
    end

    % Feedforward + correção proporcional
    F_ff    = F_res + fm * M * dv_ref_dt;
    F_cor   = Kp_cor * (v_ref(i) - v(i));
    FT_req  = F_ff + F_cor;
    FT_motor= max(0, min(FT_req, F_max));
    FT_real = min(FT_motor, F_slip_limit);   % limitado pelo solo

    FT_vec(i)  = FT_real;
    Tw_vec(i)  = FT_real * rw / n_mot;

    % (e) Dinâmica
    a_real = (FT_real - F_res) / (fm * M);
    if i < N
        v(i+1)        = max(0, min(v(i) + a_real * dt, v_target));
        pos(i+1)      = pos(i) + v(i) * dt;
        altitude(i+1) = altitude(i) + v(i) * sin(th) * dt;
    end

    % (f) Potência eléctrica com eficiência dinâmica
    eta_nom = eta_motor * eta_gear;   % 0.714 nominal
    if v(i) < 0.15 && FT_real > 0.5 * F_max
        eta_dyn = eta_stall;          % stall: corrente alta, pouca velocidade
    else
        eta_dyn = eta_nom;
    end

    P_mec_i = FT_real * v(i);
    if v(i) > 1e-6
        P_motor_elec = P_mec_i / eta_dyn;
    else
        P_motor_elec = 0;
    end
    P_tot_req = P_motor_elec + P_electronics;

    % Modelo Thévenin: V_bat = V_nom - I*R_int, P = V*I -> I^2*R - V_nom*I + P = 0
    delta_th = V_nom^2 - 4 * R_int * P_tot_req;
    if delta_th >= 0
        I_bat_i = (V_nom - sqrt(delta_th)) / (2 * R_int);
    else
        I_bat_i = V_nom / (2 * R_int);   % saturação
    end
    V_bat_i  = V_nom - I_bat_i * R_int;
    P_elec_i = V_bat_i * I_bat_i;

    I_bat_vec(i)  = I_bat_i;
    V_bat_vec(i)  = V_bat_i;
    P_elec_vec(i) = P_elec_i;

    if i > 1
        E_Wh_vec(i) = E_Wh_vec(i-1) + P_elec_i * dt / 3600;
        Ah_vec(i)   = Ah_vec(i-1)   + I_bat_i  * dt / 3600;
    end
end

%% =========================================================================
%  7) RESULTADOS E DIMENSIONAMENTO
% =========================================================================
[alt_max, i_altmax] = max(altitude);
[alt_min, i_altmin] = min(altitude);
[FT_max,  i_FTmax]  = max(FT_vec);
[Tw_peak, i_Twpk]  = max(Tw_vec);
[I_peak,  i_Ipk]   = max(I_bat_vec);
V_bat_min = min(V_bat_vec(V_bat_vec > 0));

C_ah_consumed = Ah_vec(end);
C_ah_min      = C_ah_consumed / DOD_max;
C_ah_rec      = C_ah_min * 1.20;

v_thresh     = 0.02;
stop_total_s = sum(v < v_thresh) * dt;

fprintf('\n============================================================\n');
fprintf('  ERC NAVIGATION — v_target=%.1f m/s | V_bat=%gV\n', v_target, V_nom);
fprintf('============================================================\n');
fprintf('  Distancia total           : %8.1f m\n',   pos(end));
fprintf('  Velocidade media geral    : %8.3f m/s\n', pos(end)/t_total);
fprintf('  Velocidade media em mov.  : %8.3f m/s\n', pos(end)/max(t_total-stop_total_s,1));
fprintf('  Velocidade maxima         : %8.3f m/s\n', max(v));
fprintf('------------------------------------------------------------\n');
fprintf('  Paragens programadas      : %8d\n',       n_stops);
fprintf('  Tempo total parado        : %8.0f s  (%.1f min)\n', stop_total_s, stop_total_s/60);
fprintf('------------------------------------------------------------\n');
fprintf('  Altitude maxima (vulcao)  : %+8.2f m  @ t=%.0f s\n', alt_max, t(i_altmax));
fprintf('  Altitude minima (crater)  : %+8.2f m  @ t=%.0f s\n', alt_min, t(i_altmin));
fprintf('  Slope max subida          : %+8.1f deg\n', max(theta_log));
fprintf('  Slope max descida         : %+8.1f deg\n', min(theta_log));
fprintf('------------------------------------------------------------\n');
fprintf('  FT maxima (worst case)    : %8.1f N\n',   FT_max);
fprintf('  Slope @ FT max            : %+8.1f deg\n', theta_log(i_FTmax));
fprintf('  Binario/motor (peak)      : %8.2f Nm  (Tw_max=%.0f Nm)\n', Tw_peak, Tw_max);
fprintf('------------------------------------------------------------\n');
fprintf('  Corrente pico             : %8.1f A   @ t=%.0f s\n', I_peak, t(i_Ipk));
fprintf('  Tensao minima (sag)       : %8.2f V   (nom=%gV, R_int=%.2f Ohm)\n', V_bat_min, V_nom, R_int);
fprintf('------------------------------------------------------------\n');
fprintf('  Energia total consumida   : %8.2f Wh\n',  E_Wh_vec(end));
fprintf('  Carga extraida (Ah)       : %8.2f Ah\n',  C_ah_consumed);
fprintf('  Potencia media            : %8.1f W\n',   mean(P_elec_vec));
fprintf('  Potencia maxima           : %8.1f W\n',   max(P_elec_vec));
fprintf('------------------------------------------------------------\n');
fprintf('  Capacidade min bateria    : %8.2f Ah  (DOD=%.0f%%)\n', C_ah_min, DOD_max*100);
fprintf('  Pack recomendado (+20%%)   : %8.2f Ah @ %gV\n', C_ah_rec, V_nom);
fprintf('============================================================\n\n');

%% =========================================================================
%  8) PLOTS — 6 subplots (inline, sem funcoes locais)
% =========================================================================
t_min = t / 60;

% Zonas geológicas (em TEMPO — aproximado, para visualização)
zona_t   = [0 120; 120 600; 600 1350; 1350 2100; 2100 2900; 2900 3600];
zona_cor = [0.85 0.93 1.00;   % Egress
            0.75 1.00 0.78;   % Dunes + Crater A + Inselberg
            1.00 0.92 0.65;   % Volcano zone
            0.90 0.78 1.00;   % Delta + Crater C + Outcrops
            0.78 0.95 0.85;   % Return path
            0.88 0.92 1.00];  % Final approach
zona_nome = {'Egress'; 'Dunes+CraterA+Inselberg'; 'Volcano'; ...
             'Delta+CraterC+Outcrops'; 'Return'; 'Base approach'};
n_zonas = size(zona_t, 1);

% Patches de paragem
stop_patch_t = zeros(n_stops, 2);
for s = 1:n_stops
    stop_patch_t(s,:) = [stop_list(s,1)/60, (stop_list(s,1)+stop_list(s,2))/60];
end

figure('Name', sprintf('ERC Navigation | %gV | v_max=%.1f m/s | n_mot=%d x %.0fNm', ...
       V_nom, v_target, n_mot, Tw_max), ...
       'Position', [20 20 1600 950], 'Color', 'white');

% ─── Subplot 1: Perfil de Elevação ─────────────────────────────────────────
subplot(2,3,1);
hold on;
alt_lo = alt_min - 0.3;
alt_hi = alt_max + 0.3;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [alt_lo alt_lo alt_hi alt_hi], zona_cor(z,:), 'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [alt_lo alt_lo alt_hi alt_hi], [0.72 0.72 0.72], 'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60, '--k', 'Alpha',0.20, 'LineWidth',0.8);
end
area(t_min, altitude, alt_lo, 'FaceColor',[0.55 0.35 0.12],'FaceAlpha',0.22,'EdgeColor','none');
plot(t_min, altitude, '-', 'Color',[0.30 0.15 0.02], 'LineWidth',2.2);
yline(0, '--k', 'Alpha',0.35, 'LineWidth',0.8);
plot(t_min(i_altmax), alt_max, 'v', 'Color',[0.80 0 0],'MarkerFaceColor',[0.80 0 0],'MarkerSize',8);
text(t_min(i_altmax)+0.3, alt_max+0.04, sprintf('+%.2f m (vulcao)',alt_max), 'FontSize',8,'Color',[0.70 0 0],'FontWeight','bold');
plot(t_min(i_altmin), alt_min, '^', 'Color',[0 0 0.80],'MarkerFaceColor',[0 0 0.80],'MarkerSize',8);
text(t_min(i_altmin)+0.3, alt_min-0.06, sprintf('%.2f m (crater)',alt_min), 'FontSize',8,'Color',[0 0 0.70],'FontWeight','bold');
xlim([0 60]); ylim([alt_lo alt_hi]);
xlabel('Tempo (min)'); ylabel('Altitude (m)');
title('Perfil de Elevacao — Mars Yard ERC (distance-based)','FontWeight','bold');
legend([zona_nome; {'Paragem'}], 'Location','best','FontSize',6.5);
grid on; grid minor;

% ─── Subplot 2: Inclinação ──────────────────────────────────────────────────
subplot(2,3,2);
hold on;
th_max_plot = max(abs(theta_log)) * 1.20;
th_lo = -th_max_plot; th_hi = th_max_plot;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [th_lo th_lo th_hi th_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [th_lo th_lo th_hi th_hi],[0.72 0.72 0.72],'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.20,'LineWidth',0.8);
end
area(t_min, max(theta_log,0), 0,'FaceColor',[0.85 0.20 0.20],'FaceAlpha',0.28,'EdgeColor','none');
area(t_min, min(theta_log,0), 0,'FaceColor',[0.20 0.40 0.85],'FaceAlpha',0.28,'EdgeColor','none');
plot(t_min, theta_log, 'k-','LineWidth',1.0);
th_erc_limit = 25;
yline( th_erc_limit,'--','Color',[0.8 0 0],'LineWidth',1.2,'Alpha',0.65);
yline(-th_erc_limit,'--','Color',[0 0 0.8],'LineWidth',1.2,'Alpha',0.65);
yline(0,'-k','Alpha',0.25,'LineWidth',0.8);
text(0.5, th_erc_limit*1.05, sprintf('+%g deg ERC lim',th_erc_limit),'FontSize',7.5,'Color',[0.7 0 0]);
text(0.5,-th_erc_limit*1.12, sprintf('-%g deg ERC lim',th_erc_limit),'FontSize',7.5,'Color',[0 0 0.7]);
xlim([0 60]); ylim([th_lo th_hi]);
xlabel('Tempo (min)'); ylabel('Inclinacao (deg)');
title(sprintf('Inclinacao — max %.1f deg (dist-based)', max(abs(theta_log))),'FontWeight','bold');
legend({'Subida','Descida','theta(t)'},'Location','best','FontSize',8);
grid on; grid minor;

% ─── Subplot 3: Velocidade ──────────────────────────────────────────────────
subplot(2,3,3);
hold on;
v_hi = v_target * 1.22;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 v_hi v_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 v_hi v_hi],[0.72 0.72 0.72],'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.20,'LineWidth',0.8);
end
plot(t_min, v_ref,'--','Color',[0.65 0.65 0.65],'LineWidth',1.0);
plot(t_min, v,'b-','LineWidth',1.8);
yline(v_target,'--','Color',[0 0.45 0.8],'LineWidth',1.2);
text(0.5, v_target*1.07, sprintf('v_{max}=%.1f m/s',v_target),'FontSize',8,'Color',[0 0.35 0.7]);
v_mean_mov = pos(end) / max(t_total - stop_total_s, 1);
text(58, v_mean_mov * 0.86, sprintf('v_{mov}=%.2f m/s',v_mean_mov), ...
     'HorizontalAlignment','right','FontSize',8,'Color',[0 0 0.55]);
xlim([0 60]); ylim([0 v_hi]);
xlabel('Tempo (min)'); ylabel('v (m/s)');
title('Velocidade Linear  [cinza = v_{ref} rampa]','FontWeight','bold');
legend({'v_{ref}','v(t)'},'Location','best','FontSize',8);
grid on; grid minor;

% ─── Subplot 4: Corrente e Voltage Sag ─────────────────────────────────────
subplot(2,3,4);
hold on;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 I_peak*1.18 I_peak*1.18], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 I_peak*1.18 I_peak*1.18],[0.72 0.72 0.72],'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.20,'LineWidth',0.8);
end
yyaxis left;
plot(t_min, I_bat_vec,'r-','LineWidth',1.2);
ylabel('Corrente (A)');
ylim([0 I_peak*1.18]);
[~, iPk2] = max(I_bat_vec);
text(t_min(iPk2)+0.4, I_peak*0.94, sprintf('%.1f A peak',I_peak),'FontSize',8,'Color',[0.7 0 0],'FontWeight','bold');
yyaxis right;
plot(t_min, V_bat_vec,'b-','LineWidth',1.2);
ylabel('Tensao Bateria (V)');
ylim([V_nom - I_peak*R_int*1.5,  V_nom + 0.5]);
text(0.5, V_bat_min - 0.3, sprintf('V_{min}=%.1fV (sag=%.1fV)',V_bat_min, V_nom-V_bat_min), ...
     'FontSize',7.5,'Color',[0 0 0.65]);
xlim([0 60]);
xlabel('Tempo (min)');
title(sprintf('Corrente + Voltage Sag  [R_{int}=%.2f Ohm]',R_int),'FontWeight','bold');
grid on; grid minor;

% ─── Subplot 5: Potência Eléctrica ─────────────────────────────────────────
subplot(2,3,5);
hold on;
P_hi = max(P_elec_vec)*1.18;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 P_hi P_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 P_hi P_hi],[0.72 0.72 0.72],'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.20,'LineWidth',0.8);
end
plot(t_min, P_elec_vec,'r-','LineWidth',1.2);
yline(mean(P_elec_vec),'--m','LineWidth',1.3);
text(58, mean(P_elec_vec)*1.07, sprintf('P_{med}=%.0f W',mean(P_elec_vec)), ...
     'HorizontalAlignment','right','FontSize',8,'Color',[0.7 0 0.7]);
[Pmax_v, iPmax] = max(P_elec_vec);
plot(t_min(iPmax), Pmax_v,'r^','MarkerFaceColor','red','MarkerSize',7);
text(t_min(iPmax)+0.4, Pmax_v*0.93, sprintf('%.0f W',Pmax_v),'FontSize',8,'Color',[0.7 0 0],'FontWeight','bold');
yline(P_electronics,':','Color',[0.55 0.55 0.55],'LineWidth',1.0);
text(0.5, P_electronics*1.12, sprintf('P_{standby}=%dW',P_electronics),'FontSize',7.5,'Color',[0.4 0.4 0.4]);
xlim([0 60]); ylim([0 P_hi]);
xlabel('Tempo (min)'); ylabel('P (W)');
title('Potencia Electrica Instantanea','FontWeight','bold');
grid on; grid minor;

% ─── Subplot 6: Energia + Crr(pos) ────────────────────────────────────────
subplot(2,3,6);
hold on;
E_hi = E_Wh_vec(end)*1.18;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 E_hi E_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 E_hi E_hi],[0.72 0.72 0.72],'EdgeColor','none','FaceAlpha',0.50);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.20,'LineWidth',0.8);
end
yyaxis left;
plot(t_min, E_Wh_vec,'-','Color',[0.42 0 0.60],'LineWidth',2.0);
ylabel('Energia Consumida (Wh)');
ylim([0 E_hi]);
text(58, E_Wh_vec(end)*0.84, sprintf('%.1f Wh',E_Wh_vec(end)),'HorizontalAlignment','right', ...
     'FontSize',11,'Color',[0.32 0 0.50],'FontWeight','bold');
text(58, E_Wh_vec(end)*0.73, sprintf('%.2f Ah',Ah_vec(end)),'HorizontalAlignment','right', ...
     'FontSize',9,'Color',[0.32 0 0.50]);
yyaxis right;
plot(t_min, Crr_log,'--','Color',[0.60 0.38 0.10],'LineWidth',1.5);
ylabel('Crr','Color',[0.55 0.33 0.08]);
ylim([0.06 0.24]);
yticks([0.08 0.10 0.12 0.15 0.18]);
% Crr zone labels
t_soft_mid = t(find(Crr_log == Crr_soft, 1, 'first') + ...
               round((sum(Crr_log==Crr_soft)/2))) / 60;
t_rock_mid = t(find(Crr_log == Crr_rock, 1, 'first') + ...
               round((sum(Crr_log==Crr_rock)/2))) / 60;
if ~isempty(t_soft_mid) && t_soft_mid > 0 && t_soft_mid < 60
    text(t_soft_mid, 0.200,'Delta (Crr=0.18)','FontSize',7,'Color',[0.50 0.28 0.04],'HorizontalAlignment','center');
end
if ~isempty(t_rock_mid) && t_rock_mid > 0 && t_rock_mid < 60
    text(t_rock_mid, 0.092,'Rocks (Crr=0.10)','FontSize',7,'Color',[0.50 0.28 0.04],'HorizontalAlignment','center');
end
xlim([0 60]);
xlabel('Tempo (min)');
title('Energia + Crr(t)','FontWeight','bold');
legend({'Energia (Wh)','Crr(t)'},'Location','northwest','FontSize',8);
grid on; grid minor;

% ─── Título global ──────────────────────────────────────────────────────────
line1 = sprintf('ERC Navigation — Rover %.0fkg | %gV | eta=%.0f%%nom/%.0f%%stall | mu_soil=%.2f | v_{max}=%.1fm/s', ...
                M, V_nom, eta_motor*eta_gear*100, eta_stall*100, mu_soil, v_target);
line2 = sprintf('Dist:%.0fm | E:%.1fWh | %.2fAh | Alt:[%.1f;+%.1f]m | Slope:[%.0f;+%.0f]deg | P_max:%.0fW | I_peak:%.0fA | Bat:%.1fAh@%gV | Stops:%d', ...
                pos(end), E_Wh_vec(end), Ah_vec(end), alt_min, alt_max, ...
                min(theta_log), max(theta_log), max(P_elec_vec), I_peak, ...
                C_ah_rec, V_nom, n_stops);
sgtitle({line1; line2},'FontSize',10,'FontWeight','bold');
