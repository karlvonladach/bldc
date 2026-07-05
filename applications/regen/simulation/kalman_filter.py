import numpy as np
import matplotlib.pyplot as plt

# --- 1. SZIMULÁCIÓS PARAMÉTEREK ---
dt = 0.01  # Időlépés (100 Hz-es mintavételezés)
N = 2000   # Szimulációs lépések száma (20 másodperc)
t = np.linspace(0, N*dt, N)

m = 100.0  # Effektív tömeg (bringás + ebike = 100 kg)

# --- 2. VALÓS (Zajmentes) JELEK GENERÁLÁSA ---
# Emberi nyomaték: 15 Nm alapérték + 12 Nm lüktetés (szinusz 2.5 Hz-en, a lábak ritmusa)
true_tau = 15.0 + 12.0 * np.sin(2 * np.pi * 2.5 * t)
# A szimuláció végén a bringás abbahagyja a tekerést
true_tau[t > 15.0] = 0.0

# Pedál fordulatszám (rad/s): lassan növekszik 2 rad/s-ról (~20 RPM) 8 rad/s-ra (~76 RPM)
true_omega = np.zeros(N)
for k in range(N):
    if t[k] <= 15.0:
        true_omega[k] = 2.0 + 6.0 * (t[k] / 15.0)
    else:
        true_omega[k] = 0.0 # megáll a pedál is

# Környezeti ellenállás (F_resistance): 
# Alapból 20 N (sík út, szél), majd a 7. másodpercnél jön egy emelkedő (+50 N)
true_R = np.ones(N) * 20.0
true_R[t > 7.0] = 70.0

# Sebesség kiszámítása a fizikai modell alapján (Euler integrálással)
true_v = np.zeros(N)
current_v = 0.5 # kis kezdeti sebesség, hogy ne 0-ról induljon az osztás

for k in range(1, N):
    if true_v[k-1] > 0.1 and true_omega[k-1] > 0.1:
        F_human = (true_tau[k-1] * true_omega[k-1]) / true_v[k-1]
    else:
        F_human = 0.0
    
    # m * a = F_human - F_resistance
    dv = (F_human - true_R[k-1]) / m * dt
    current_v += dv
    if current_v < 0.0: current_v = 0.0 # nem gurul hátra
    true_v[k] = current_v

# --- 3. ZAJOS MÉRÉSEK GENERÁLÁSA ---
np.random.seed(42) # reprodukálhatóság
sigma_v = 0.3      # Sebességmérő zaja (m/s)
sigma_tau = 4.0    # Nyomatékszenzor zaja (Nm)
sigma_omega = 0.5  # Pedálszenzor zaja (rad/s)

meas_v = true_v + np.random.normal(0, sigma_v, N)
meas_tau = true_tau + np.random.normal(0, sigma_tau, N)
meas_omega = true_omega + np.random.normal(0, sigma_omega, N)

# --- 4. KITERJESZTETT KÁLMÁN-SZŰRŐ (EKF) ---
# Állapotvektor: x = [v, R, tau, omega]^T
x = np.array([[0.5],  # v kezdeti tippelése
              [20.0], # R kezdeti tippelése
              [15.0], # tau kezdeti tippelése
              [2.0]]) # omega kezdeti tippelése

# Kovariancia mátrix (Kezdeti bizonytalanság)
P = np.diag([1.0, 100.0, 50.0, 5.0])

# Folyamatzaj mátrix (Q) - mennyire bízunk a modellünkben/mennyire változhatnak az állapotok
# R, tau és omega random walk-ot követ, a v-t a fizika vezérli
Q = np.diag([0.001, 5.0, 5.0, 0.1])

# Mérési zaj mátrix (R_mat) - a szenzorok ismert szórásai négyzeten
R_mat = np.diag([sigma_v**2, sigma_tau**2, sigma_omega**2])

# Eredmények tárolása
est_x = np.zeros((4, N))

for k in range(N):
    # --- PREDIKCIÓ (JÓSLÁS) ---
    v_est = max(x[0, 0], 0.1) # Védelem a 0-val osztás ellen
    R_est = x[1, 0]
    tau_est = x[2, 0]
    omega_est = x[3, 0]
    
    if omega_est > 0.1:
        F_h_est = (tau_est * omega_est) / v_est
    else:
        F_h_est = 0.0
        
    # Nemlineáris állapot-átmenet f(x)
    v_next = x[0, 0] + (dt / m) * (F_h_est - R_est)
    if v_next < 0: v_next = 0.0
        
    x_pred = np.array([[v_next],
                       [R_est],
                       [tau_est],
                       [omega_est]])
    
    # Jacobi-mátrix (F) a kovariancia transzformációhoz
    F = np.eye(4)
    if omega_est > 0.1:
        df_dv = 1.0 - (dt / m) * (tau_est * omega_est) / (v_est**2)
        df_dtau = (dt / m) * (omega_est / v_est)
        df_domega = (dt / m) * (tau_est / v_est)
    else:
        df_dv = 1.0
        df_dtau = 0.0
        df_domega = 0.0
        
    F[0, 0] = df_dv
    F[0, 1] = -dt / m
    F[0, 2] = df_dtau
    F[0, 3] = df_domega
    
    # Kovariancia predikció
    P_pred = F @ P @ F.T + Q
    
    # --- KORREKCIÓ (FRISSÍTÉS) ---
    # Mérési mátrix (H) - mivel v, tau, omega közvetlenül mértek:
    H = np.array([[1, 0, 0, 0],  # méri a v-t
                  [0, 0, 1, 0],  # méri a tau-t
                  [0, 0, 0, 1]]) # méri az omega-t
    
    # Várható mérés
    z_pred = np.array([[x_pred[0, 0]],
                       [x_pred[2, 0]],
                       [x_pred[3, 0]]])
    
    # Aktuális mérés
    z_meas = np.array([[meas_v[k]],
                       [meas_tau[k]],
                       [meas_omega[k]]])
    
    # Kálmán-nyereség számítása
    S = H @ P_pred @ H.T + R_mat
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # Állapot és kovariancia frissítése
    x = x_pred + K @ (z_meas - z_pred)
    P = (np.eye(4) - K @ H) @ P_pred
    
    # Mentés
    est_x[:, k] = x.ravel()

# --- 5. VIZUALIZÁCIÓ ---
plt.figure(figsize=(12, 10))

# 1. Grafikon: Pedál Nyomaték
plt.subplot(4, 1, 1)
plt.plot(t, meas_tau, alpha=0.3, color='gray', label='Zajos mérés (Szenzor)')
plt.plot(t, true_tau, 'g-', linewidth=2, label='Valós (Tiszta)')
plt.plot(t, est_x[2, :], 'r--', linewidth=2, label='EKF becsült / szűrt')
plt.ylabel('Nyomaték [Nm]')
plt.title('E-Bike Állapotbecslés és Zajszűrés Kálmán-szűrővel (EKF)')
plt.legend(loc='upper right')
plt.grid(True)

# 2. Grafikon: Pedál fordulat
plt.subplot(4, 1, 2)
plt.plot(t, meas_omega, alpha=0.3, color='gray', label='Zajos mérés')
plt.plot(t, true_omega, 'g-', linewidth=2, label='Valós')
plt.plot(t, est_x[3, :], 'r--', linewidth=2, label='EKF szűrt')
plt.ylabel('Pedálford. [rad/s]')
plt.legend(loc='upper right')
plt.grid(True)

# 3. Grafikon: Keréksebesség
plt.subplot(4, 1, 3)
plt.plot(t, meas_v, alpha=0.3, color='gray', label='Zajos mérés')
plt.plot(t, true_v, 'g-', linewidth=2, label='Valós')
plt.plot(t, est_x[0, :], 'r--', linewidth=2, label='EKF szűrt')
plt.ylabel('Sebesség [m/s]')
plt.legend(loc='upper right')
plt.grid(True)

# 4. Grafikon: Becsült Ellenállás (A lényeg!)
plt.subplot(4, 1, 4)
plt.plot(t, true_R, 'g-', linewidth=2, label='Valós ellenállás (Domborzat)')
plt.plot(t, est_x[1, :], 'b-', linewidth=2, label='EKF Becsült Ellenállás')
plt.axhline(20, color='gray', linestyle=':', alpha=0.7)
plt.axhline(70, color='gray', linestyle=':', alpha=0.7)
plt.xlabel('Idő [másodperc]')
plt.ylabel('Ellenállás [N]')
plt.legend(loc='upper right')
plt.grid(True)

plt.tight_layout()
plt.show()