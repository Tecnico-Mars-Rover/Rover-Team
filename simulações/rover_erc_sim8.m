clc;
clear;
close all;

%% =========================================================================
%  SIMULAÇÃO DE NAVEGAÇÃO ERC - ROVER 75 kg  (Navigation Task)
%  Objetivo: dimensionamento de motores e bateria
%
%  Modelo de movimento:
%    - Velocidade alvo = 1 m/s (máximo regulamento ERC)
%    - Aceleração/desaceleração realista com rampa limitada fisicamente
%    - Paragens aleatórias realistas (~13 ao longo de 60 min)
%    - Inércia representada via fm (massa rotacional equivalente)
%    - Força de tração saturada em F_max dos motores
%
%  Referências:
%    - Losiak et al. (2023) "Mars Yard Design During ERC 2020-2022"
%    - ERC Rulebook: v_max=1 m/s, slope ~25-30 deg, Crr=0.10-0.18
%    - Análise de Eficiências - Equipa Rover IST N3E (2026)
%    - Projeto VIENA, IST 2018-2019
% =========================================================================

%% =========================================================================
%  1) PARÂMETROS — ALTERAR AQUI
% =========================================================================

% --- Veículo ---
M             = 75;    % Massa total do rover               [kg]
fm            = 1.1;  % Fator de massa rotacional          [-]
rw            = 0.125; % Raio da roda                       [m]
n_mot         = 4;     % Número de motores (4WD)            [-]
Tw_max        = 35;    % Binário máximo por motor           [Nm]

% --- Cinemática ---
v_target      = 1.0;   % Velocidade alvo (max regulamento ERC)  [m/s]
a_accel       = 0.5;   % Aceleração máx da rampa de referência  [m/s^2]
a_decel       = 1.0;   % Desaceleração máx (travagem)           [m/s^2]
%  Nota: a aceleração física máxima dos motores é F_max/(fm*M) = 14.2 m/s^2
%  Usamos 0.5 m/s^2 para representar o controlo suave real do rover

% --- Terreno ---
theta_max_deg = 45;    % <<< INCLINAÇÃO MÁXIMA [graus] >>>
                       %     Escala todo o perfil proporcionalmente.
                       %     Valores ERC típicos: 10, 15, 20, 25, 30

Crr_sand      = 0.12;  % Coef. rolamento areia/granular         [-]
Crr_soft      = 0.32;  % Coef. rolamento solo mole (delta)      [-]
Crr_rock      = 0.10;  % Coef. rolamento rocha firme            [-]

% --- Energia ---
g             = 9.81;  % Gravidade                              [m/s^2]
eta           = 0.5; % Eficiência BLDC 24V (bateria -> roda)  [-]
P_electronics = 50;    % Consumo base eletrónica (consumo de todos os outros sistemas não relacionados com tração)   [W]
V_bat         = 48;    % Tensão nominal do pack                 [V]
DOD_max       = 0.80;  % Profundidade descarga máxima           [-]

% --- Simulação ---
dt            = 0.1;   % Passo de tempo                         [s]
t_total       = 3600;  % Duração total (60 min)                 [s]
rng_seed      = 42;    % Semente para reprodutibilidade (garante que os numeros aletorios são sempre os mesmos)      [-]

%% =========================================================================
%  2) VETOR DE TEMPO
% =========================================================================
t = 0 : dt : t_total;
N = length(t);

%% =========================================================================
%  3) PERFIL DE TERRENO — keyframes normalizados + PCHIP
%
%  kf_theta_norm em [-1, +1]:
%    +1.0 = pico máximo positivo (topo do vulcão)
%    -1.0 = pico máximo negativo (descida mais íngreme do crater)
%  theta_deg(t) = kf_theta_norm * theta_max_deg
% =========================================================================

kf_t = [ ...
  0  40  90  150  220  280  300  ...
  340  390  440  490  540  ...
  560  600  640  670  700  740  780  820  860  ...
  880  930  980  1020  ...
  1060  1100  1150  1200  ...
  1250  1300  1380  ...
  1420  1480  1540  1610  1680  1740  ...
  1770  1840  1910  1980  ...
  2010  2070  2130  2200  ...
  2230  2280  2330  2380  2430  2490  ...
  2520  2600  2680  2760  2840  2920  3000  ...
  3050  3150  3250  3350  3450  3550  3600 ];

kf_theta_norm = [ ...
  0.000  -0.143  -0.286  -0.429  -0.357  -0.214  -0.071  ...
  0.071   0.179   0.107   0.000  -0.107  ...
 -0.179  -0.643  -0.893  -0.786  -0.429  -0.071   0.000   0.500   0.643  ...
  0.643   0.786   0.536   0.179  ...
  0.143   0.286   0.107   0.000  ...
 -0.143   0.071  -0.071  ...
  0.214   0.500   0.786   0.929   1.000   0.786  ...
 -0.571  -1.000  -0.714  -0.286  ...
 -0.143   0.250   0.143  -0.107  ...
 -0.143  -0.500  -0.714  -0.571  -0.214   0.286  ...
  0.286  -0.357   0.429  -0.286   0.250  -0.357   0.143  ...
  0.107   0.214   0.143   0.071   0.036   0.000   0.000 ];

% --- Validações (coisas como dois vetores terem o mesmo numero de pontos se não cancela o codigo) ---
assert(length(kf_t) == length(kf_theta_norm), ...
    'kf_t (%d) != kf_theta_norm (%d)', length(kf_t), length(kf_theta_norm));
assert(all(diff(kf_t) > 0), 'kf_t nao e estritamente crescente');
assert(max(abs(kf_theta_norm)) <= 1.0 + 1e-9, 'kf_theta_norm fora de [-1,+1]');

% --- Escalar e interpolar (transforma os pontos definidos manualmente → terreno contínuo) ---
theta_base_deg = interp1(kf_t, kf_theta_norm * theta_max_deg, t, 'pchip');

% --- Micro-rugosidades do solo ---
rng(rng_seed);
noise_amp  = theta_max_deg * 0.05;  % 45 * 0.05 = 2.25º de inregularidade max
noise_hf   = movmean(noise_amp * randn(1, N), 20);
noise_mf   = movmean(noise_amp * 0.4 * sin(2*pi*t/55 + randn(1,N)*0.4), 50);
theta_deg_sim = max(-theta_max_deg, min(theta_max_deg, theta_base_deg + noise_hf + noise_mf));
theta_rad     = deg2rad(theta_deg_sim);

% --- Crr variável por zona ---
Crr_t = Crr_sand * ones(1, N);
Crr_t(t >= 2010 & t <= 2200) = Crr_soft;
Crr_t(t >= 2520 & t <= 3000) = Crr_rock;

%% =========================================================================
%  4) AGENDA DE PARAGENS — stop/go realista
%
%  Paragens geradas com espaçamento e duração aleatórios (semente fixa).
%  Representa: obstáculos, reposicionamento, espera de comando, etc.
%  Fora das paragens o rover acelera suavemente até v_target com rampa
%  limitada a a_accel. Ao entrar numa paragem desacelera a a_decel.
% =========================================================================
rng(rng_seed + 1);  % semente separada para nao interferir com ruido do terreno

stop_list = [];     % colunas: [t_inicio_s, duracao_s]
t_probe   = 120;    % nao parar nos primeiros 2 min (egress)
while t_probe < 3400
    gap_s   = randi([150 350]);
    t_probe = t_probe + gap_s;
    if t_probe >= 3400; break; end
    dur_s = randi([15 50]);
    stop_list(end+1, :) = [t_probe, dur_s]; %#ok<AGROW>
end
n_stops = size(stop_list, 1);

% Vetor booleano: is_stopped(i) = true quando o rover deve desacelerar para 0
is_stopped = false(1, N);
for s = 1 : n_stops
    t_s = stop_list(s, 1);
    t_e = t_s + stop_list(s, 2);
    is_stopped(t >= t_s & t <= t_e) = true;
end

%% =========================================================================
%  5) PRÉ-ALOCAÇÃO
% =========================================================================
v        = zeros(1, N);   % velocidade linear [m/s]
v_ref    = zeros(1, N);   % referência de velocidade (rampa) [m/s]
pos      = zeros(1, N);   % posição acumulada [m]
FT_vec   = zeros(1, N);   % força de tração total [N]
P_elec   = zeros(1, N);   % potência eléctrica [W]
E_Wh     = zeros(1, N);   % energia acumulada [Wh]
altitude = zeros(1, N);   % altitude [m]

%% =========================================================================
%  6) LOOP DE SIMULAÇÃO — integração de Euler com rampa de velocidade
%
%  Modelo de controlo em dois níveis:
%
%  Nível 1 — Gerador de referência (rampa):
%    Se em movimento : v_ref += a_accel * dt  (sobe até v_target)
%    Se em paragem   : v_ref -= a_decel * dt  (desce até 0)
%
%  Nível 2 — Força de tração (feedforward + correção P):
%    F_ff  = F_res + fm*M * (dv_ref/dt)   feedforward de aceleração
%    F_cor = Kp * (v_ref - v)             correção proporcional
%    FT    = clip(F_ff + F_cor, 0, F_max)
%
%  Equação de movimento:
%    fm*M * dv/dt = FT - F_slope - F_roll
% =========================================================================
F_max  = (Tw_max / rw) * n_mot;   % Força máxima total: 1120 N
Kp_cor = fm * M * 2.0;            % Ganho P pequeno (~158 N/(m/s))

for i = 1 : N

    th  = theta_rad(i);
    Crr = Crr_t(i);

    % Forças resistivas
    F_slope = M * g * sin(th);
    F_roll  = M * g * abs(cos(th)) * Crr;
    F_res   = F_slope + F_roll;

    % Gerador de referência de velocidade (rampa)
    if is_stopped(i)
        dv_ref_dt = -a_decel;
    else
        dv_ref_dt = +a_accel;
    end
    if i < N
        v_ref(i+1) = max(0, min(v_target, v_ref(i) + dv_ref_dt * dt));
    end

    % Feedforward + correção proporcional
    F_ff      = F_res + fm * M * dv_ref_dt;
    F_cor     = Kp_cor * (v_ref(i) - v(i));
    FT_vec(i) = max(0, min(F_ff + F_cor, F_max));

    % Equação de movimento: fm*M*dv/dt = FT - F_res
    a_real = (FT_vec(i) - F_res) / (fm * M);

    % Integração de Euler
    if i < N
        v(i+1)        = max(0, min(v(i) + a_real * dt, v_target));
        pos(i+1)      = pos(i) + v(i) * dt;
        altitude(i+1) = altitude(i) + v(i) * sin(th) * dt;
    end

    % Potência eléctrica
    P_mec_i   = FT_vec(i) * v(i);
    if v(i) > 1e-6
        P_elec(i) = P_mec_i / eta + P_electronics;
    else
        P_elec(i) = P_electronics;  % parado: apenas eletrónica base
    end

    % Energia acumulada
    if i > 1
        E_Wh(i) = E_Wh(i-1) + P_elec(i) * dt / 3600;
    end
end

%% =========================================================================
%  7) RESULTADOS E DIMENSIONAMENTO
% =========================================================================
[alt_max, i_altmax] = max(altitude);
[alt_min, i_altmin] = min(altitude);
[FT_max,  i_FTmax]  = max(FT_vec);
Tw_worst  = FT_max * rw / n_mot;
I_bat_max = max(P_elec) / V_bat;
C_ah_min  = E_Wh(end) / (V_bat * DOD_max);
C_ah_rec  = C_ah_min * 1.20;

% Tempo efectivo parado (v < 2 cm/s)
v_thresh     = 0.02;
stop_total_s = sum(v < v_thresh) * dt;

fprintf('\n============================================================\n');
fprintf('  ERC NAVIGATION — theta_max=%g deg | v_target=%.1f m/s\n', theta_max_deg, v_target);
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
fprintf('  Slope max subida          : %+8.1f deg\n', max(theta_deg_sim));
fprintf('  Slope max descida         : %+8.1f deg\n', min(theta_deg_sim));
fprintf('------------------------------------------------------------\n');
fprintf('  FT maxima (worst case)    : %8.1f N   @ theta=%.1f deg\n', FT_max, theta_deg_sim(i_FTmax));
fprintf('  Binario/motor (worst)     : %8.2f Nm  (Tw_max=%.0f Nm)\n', Tw_worst, Tw_max);
fprintf('  Corrente bat %gV (max)    : %8.1f A\n',   V_bat, I_bat_max);
fprintf('------------------------------------------------------------\n');
fprintf('  Energia total consumida   : %8.2f Wh\n',  E_Wh(end));
fprintf('  Potencia media            : %8.1f W\n',   mean(P_elec));
fprintf('  Potencia maxima           : %8.1f W\n',   max(P_elec));
fprintf('------------------------------------------------------------\n');
fprintf('  Capacidade min bateria    : %8.2f Ah  (DOD=%.0f%%)\n', C_ah_min, DOD_max*100);
fprintf('  Pack recomendado (+20%%)   : %8.2f Ah @ %gV\n', C_ah_rec, V_bat);
fprintf('============================================================\n\n');

%% =========================================================================
%  8) PLOTS — 6 subplots, sombreamento inline (sem funcoes locais)
% =========================================================================
t_min = t / 60;

zona_t   = [0 300; 300 1200; 1200 2000; 2000 3000; 3000 3600];
zona_cor = [0.80 0.90 1.00;
            0.75 1.00 0.75;
            1.00 0.92 0.65;
            0.90 0.78 1.00;
            0.78 0.95 0.78];
zona_nome = {'Egress'; 'Traverse I (crater+inselberg)'; ...
             'Vulcao + delta'; 'Crater II + outcrops'; 'Return'};
n_zonas = size(zona_t, 1);

% Pré-calcular patches de paragem em minutos
stop_patch_t = zeros(n_stops, 2);
for s = 1:n_stops
    stop_patch_t(s,:) = [stop_list(s,1)/60, (stop_list(s,1)+stop_list(s,2))/60];
end

figure('Name', sprintf('ERC Navigation | theta_max=%g deg | v_target=%.1f m/s', ...
       theta_max_deg, v_target), ...
       'Position', [20 20 1560 960], 'Color', 'white');

% ─── Subplot 1: Perfil de Elevação ────────────────────────────────────────
subplot(2,3,1);
hold on;
alt_lo = alt_min - 0.5;
alt_hi = alt_max + 0.5;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [alt_lo alt_lo alt_hi alt_hi], zona_cor(z,:), 'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [alt_lo alt_lo alt_hi alt_hi], [0.7 0.7 0.7], 'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60, '--k', 'Alpha',0.22, 'LineWidth',0.8);
end
area(t_min, altitude, alt_lo, 'FaceColor',[0.55 0.35 0.12],'FaceAlpha',0.22,'EdgeColor','none');
plot(t_min, altitude, '-', 'Color',[0.35 0.18 0.02], 'LineWidth',2.2);
yline(0, '--k', 'Alpha',0.4, 'LineWidth',0.8);
plot(t_min(i_altmax), alt_max, 'v', 'Color',[0.8 0 0],'MarkerFaceColor',[0.8 0 0],'MarkerSize',8);
text(t_min(i_altmax)+0.3, alt_max+0.05, sprintf('+%.2f m',alt_max), 'FontSize',8,'Color',[0.7 0 0],'FontWeight','bold');
plot(t_min(i_altmin), alt_min, '^', 'Color',[0 0 0.8],'MarkerFaceColor',[0 0 0.8],'MarkerSize',8);
text(t_min(i_altmin)+0.3, alt_min-0.08, sprintf('%.2f m',alt_min), 'FontSize',8,'Color',[0 0 0.7],'FontWeight','bold');
xlim([0 60]); ylim([alt_lo alt_hi]);
xlabel('Tempo (min)'); ylabel('Altitude (m)');
title('Perfil de Elevacao — Mars Yard ERC','FontWeight','bold');
legend([zona_nome; {'Paragem'}],'Location','best','FontSize',7);
grid on; grid minor;

% ─── Subplot 2: Inclinação ────────────────────────────────────────────────
subplot(2,3,2);
hold on;
th_lo = -theta_max_deg*1.18;
th_hi =  theta_max_deg*1.18;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [th_lo th_lo th_hi th_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [th_lo th_lo th_hi th_hi],[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.22,'LineWidth',0.8);
end
area(t_min, max(theta_deg_sim,0), 0,'FaceColor',[0.85 0.20 0.20],'FaceAlpha',0.28,'EdgeColor','none');
area(t_min, min(theta_deg_sim,0), 0,'FaceColor',[0.20 0.40 0.85],'FaceAlpha',0.28,'EdgeColor','none');
plot(t_min, theta_deg_sim, 'k-','LineWidth',1.1);
yline( theta_max_deg,'--','Color',[0.8 0 0],'LineWidth',1.2,'Alpha',0.7);
yline(-theta_max_deg,'--','Color',[0 0 0.8],'LineWidth',1.2,'Alpha',0.7);
yline(0,'-k','Alpha',0.3,'LineWidth',0.8);
text(0.5, theta_max_deg*1.04, sprintf('+%g deg',theta_max_deg),'FontSize',8,'Color',[0.7 0 0]);
text(0.5,-theta_max_deg*1.10, sprintf('-%g deg',theta_max_deg),'FontSize',8,'Color',[0 0 0.7]);
xlim([0 60]); ylim([th_lo th_hi]);
xlabel('Tempo (min)'); ylabel('Inclinacao (deg)');
title(sprintf('Inclinacao do Terreno  [max = %g deg]',theta_max_deg),'FontWeight','bold');
legend({'Subida','Descida','theta(t)'},'Location','best','FontSize',8);
grid on; grid minor;

% ─── Subplot 3: Velocidade ────────────────────────────────────────────────
subplot(2,3,3);
hold on;
v_hi = v_target * 1.20;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 v_hi v_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 v_hi v_hi],[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.22,'LineWidth',0.8);
end
plot(t_min, v_ref,'--','Color',[0.65 0.65 0.65],'LineWidth',1.0);
plot(t_min, v, 'b-','LineWidth',1.8);
yline(v_target,'--','Color',[0 0.45 0.8],'LineWidth',1.2);
text(0.5, v_target*1.06, sprintf('v_{max}=%.1f m/s',v_target),'FontSize',8,'Color',[0 0.35 0.7]);
v_mean_mov = pos(end)/max(t_total-stop_total_s, 1);
text(58, v_mean_mov*0.88, sprintf('v_{med,mov}=%.2f m/s',v_mean_mov),'HorizontalAlignment','right','FontSize',8,'Color',[0 0 0.55]);
xlim([0 60]); ylim([0 v_hi]);
xlabel('Tempo (min)'); ylabel('v (m/s)');
title('Velocidade Linear  [cinza = v_{ref} rampa]','FontWeight','bold');
legend({'v_{ref}','v(t)'},'Location','best','FontSize',8);
grid on; grid minor;

% ─── Subplot 4: Posição Acumulada ─────────────────────────────────────────
subplot(2,3,4);
hold on;
pos_hi = pos(end)*1.15;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 pos_hi pos_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 pos_hi pos_hi],[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.22,'LineWidth',0.8);
end
plot(t_min, pos,'-','Color',[0.05 0.55 0.05],'LineWidth',1.8);
text(58, pos(end)*0.88, sprintf('%.0f m',pos(end)),'HorizontalAlignment','right', ...
     'FontSize',11,'Color',[0.0 0.45 0.0],'FontWeight','bold');
xlim([0 60]); ylim([0 pos_hi]);
xlabel('Tempo (min)'); ylabel('Posicao (m)');
title('Distancia Acumulada','FontWeight','bold');
grid on; grid minor;

% ─── Subplot 5: Potência Eléctrica ────────────────────────────────────────
subplot(2,3,5);
hold on;
P_hi = max(P_elec)*1.18;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 P_hi P_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 P_hi P_hi],[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.22,'LineWidth',0.8);
end
plot(t_min, P_elec,'r-','LineWidth',1.2);
yline(mean(P_elec),'--m','LineWidth',1.3);
text(58, mean(P_elec)*1.06, sprintf('Media: %.0f W',mean(P_elec)), ...
     'HorizontalAlignment','right','FontSize',8,'Color',[0.7 0 0.7]);
[Pmax_v, iPmax] = max(P_elec);
plot(t_min(iPmax), Pmax_v,'r^','MarkerFaceColor','red','MarkerSize',7);
text(t_min(iPmax)+0.4, Pmax_v*0.94, sprintf('%.0f W',Pmax_v), ...
     'FontSize',8,'Color',[0.7 0 0],'FontWeight','bold');
yline(P_electronics,':','Color',[0.5 0.5 0.5],'LineWidth',1.0);
text(0.5, P_electronics*1.10, sprintf('P_{standby}=%dW',P_electronics),'FontSize',7.5,'Color',[0.4 0.4 0.4]);
xlim([0 60]); ylim([0 P_hi]);
xlabel('Tempo (min)'); ylabel('P (W)');
title('Potencia Electrica Instantanea','FontWeight','bold');
grid on; grid minor;

% ─── Subplot 6: Energia + Crr ─────────────────────────────────────────────
subplot(2,3,6);
hold on;
E_hi = E_Wh(end)*1.18;
for z = 1:n_zonas
    patch([zona_t(z,1)/60 zona_t(z,2)/60 zona_t(z,2)/60 zona_t(z,1)/60], ...
          [0 0 E_hi E_hi], zona_cor(z,:),'EdgeColor','none','FaceAlpha',0.35);
end
for s = 1:n_stops
    patch([stop_patch_t(s,1) stop_patch_t(s,2) stop_patch_t(s,2) stop_patch_t(s,1)], ...
          [0 0 E_hi E_hi],[0.7 0.7 0.7],'EdgeColor','none','FaceAlpha',0.45);
end
for z = 2:n_zonas
    xline(zona_t(z,1)/60,'--k','Alpha',0.22,'LineWidth',0.8);
end
yyaxis left;
plot(t_min, E_Wh,'-','Color',[0.42 0 0.60],'LineWidth',2.0);
ylabel('Energia Consumida (Wh)');
ylim([0 E_hi]);
text(58, E_Wh(end)*0.85, sprintf('%.1f Wh',E_Wh(end)),'HorizontalAlignment','right', ...
     'FontSize',11,'Color',[0.32 0 0.50],'FontWeight','bold');
yyaxis right;
plot(t_min, Crr_t,'--','Color',[0.60 0.38 0.10],'LineWidth',1.5);
ylabel('Crr','Color',[0.55 0.33 0.08]);
ylim([0.06 0.24]);
yticks([0.08 0.10 0.12 0.15 0.18]);
text(mean([2010 2200])/60, 0.200,'Delta (Crr=0.18)','FontSize',7.5,'Color',[0.50 0.28 0.04],'HorizontalAlignment','center');
text(mean([2520 3000])/60, 0.094,'Rocks (Crr=0.10)','FontSize',7.5,'Color',[0.50 0.28 0.04],'HorizontalAlignment','center');
xlim([0 60]);
xlabel('Tempo (min)');
title('Energia Consumida + Crr(t)','FontWeight','bold');
legend({'Energia (Wh)','Crr variavel'},'Location','northwest','FontSize',8);
grid on; grid minor;

% ─── Título global ─────────────────────────────────────────────────────────
line1 = sprintf('ERC Navigation — Rover %.0f kg | BLDC 24V (eta=%.1f%%) | theta_max=%g deg | v_{max}=%.1f m/s', ...
                M, eta*100, theta_max_deg, v_target);
line2 = sprintf('Dist: %.0f m | E: %.1f Wh | Alt:[%.1f;+%.1f]m | Slope:[%.0f;+%.0f]deg | P_max:%.0f W | Bat:%.1f Ah @ %gV | Paragens:%d', ...
                pos(end), E_Wh(end), alt_min, alt_max, ...
                min(theta_deg_sim), max(theta_deg_sim), ...
                max(P_elec), C_ah_rec, V_bat, n_stops);
sgtitle({line1; line2},'FontSize',10.5,'FontWeight','bold');
