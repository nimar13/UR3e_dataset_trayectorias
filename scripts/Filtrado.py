# ======================================================
# CÓDIGO: Filtrado
# ======================================================

import os
import pandas as pd
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
#Parámetros del filtro Butterworth de paso bajo con filtfilt (cero fase)
FS    = 500   #Frecuencia de muestreo en Hz
FC    = 25    #Frecuencia de corte en Hz
ORDER = 4     #Orden del filtro
# ---------------------------------------------------------------------------
Wn = FC / (FS / 2)  #Frecuencia normalizada
b, a = butter(N=ORDER, Wn=Wn, btype='low', analog=False)

#Rutas de entrada y salida
# Directorios relativos (desde la raíz del repositorio)
INPUT_ROOT  = os.path.join("trayectorias_ejecutadas")
OUTPUT_ROOT = os.path.join("trayectorias_ejecutadas_filtradas")
os.makedirs(OUTPUT_ROOT, exist_ok=True)

#Procesamiento de cada interpolador y cada trayectoria
for interp_name in os.listdir(INPUT_ROOT):
    in_dir = os.path.join(INPUT_ROOT, interp_name)
    if not os.path.isdir(in_dir):
        continue
    out_dir = os.path.join(OUTPUT_ROOT, interp_name)
    os.makedirs(out_dir, exist_ok=True)

    for fname in os.listdir(in_dir):
        if not fname.lower().endswith(".csv"):
            continue

        base     = fname[:-4]
        in_path  = os.path.join(in_dir, fname)
        out_csv  = os.path.join(out_dir, f"{base}_filtrado.csv")
        out_png  = os.path.join(out_dir, f"{base}_filtrado.png")

        #Lectura del CSV original
        df = pd.read_csv(in_path)
        df_f = df.copy()

        #Columnas de posición y velocidad
        joints_pos = [f"q{i}"  for i in range(1, 7) if f"q{i}"  in df.columns]
        joints_vel = [f"qd{i}" for i in range(1, 7) if f"qd{i}" in df.columns]

        #Aplicar filtro a posiciones
        for j in joints_pos:
            df_f[j] = filtfilt(b, a, df[j].values)

        #Aplicar filtro a velocidades
        for j in joints_vel:
            df_f[j] = filtfilt(b, a, df[j].values)

        #Guardado del CSV filtrado
        df_f.to_csv(out_csv, index=False)

        #Gráfico comparativo de un segmento de 200–500 muestras
        seg = slice(200, 500)
        plt.figure(figsize=(10, 6))

        #Posiciones
        for j in joints_pos:
            plt.plot(df.index[seg], df[j].iloc[seg],    '--', alpha=0.4, label=f"{j} original")
            plt.plot(df.index[seg], df_f[j].iloc[seg],        label=f"{j} filtrado")

        #Velocidades
        for j in joints_vel:
            plt.plot(df.index[seg], df[j].iloc[seg],    '--', alpha=0.3, label=f"{j} original")
            plt.plot(df.index[seg], df_f[j].iloc[seg],        label=f"{j} filtrado")

        plt.xlabel("Índice de muestra")
        plt.ylabel("Valor")
        plt.title(f"{interp_name} → {base}: Butterworth {FC} Hz (orden {ORDER})")
        plt.legend(fontsize='small', ncol=3)
        plt.tight_layout()
        plt.savefig(out_png)
        plt.close()

print("Filtro Butterworth aplicado con éxito a posiciones y velocidades.")
print("Revisa los CSVs y gráficos en:")
print(OUTPUT_ROOT)
