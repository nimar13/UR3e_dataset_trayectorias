# UR3e_dataset_trayectorias

Este repositorio contiene un conjunto de trayectorias generadas, ejecutadas y validadas para el robot colaborativo **UR3e**, como parte del Trabajo de Fin de Grado en Ingeniería Robótica.

El dataset está organizado para facilitar su reutilización en tareas de simulación, planificación, interpolación y aprendizaje automático en robótica colaborativa.

Este repositorio incluye 500 trayectorias simuladas por interpolador. El dataset completo de 5.000 trayectorias por interpolador (25.000 en total) está disponible bajo petición o mediante enlace externo.

---

## Estructura del repositorio

├── data/ # Trayectorias simuladas 
├── trayectorias_ejecutadas/ # Resultados reales tras ejecución física
├── trayectorias_ejecutadas_filtradas/ # Resultados ejecutados y filtrados digitalmente
├── scripts/ # Scripts para generar, ejecutar y filtrar trayectorias
├── logs/ # Registro de errores e incidencias
├── docs/ # Diagramas explicativos del pipeline
├── LICENSE # Licencia MIT
└── README.md # Este documento

---

## Contenido del dataset

- Trayectorias generadas con 5 métodos de interpolación:  
  `JTRAJ`, `QUINTIC`, `QUINTIC_FUNC`, `TRAPEZOIDAL`, `TRAPEZOIDAL_FUNC`
- Cada trayectoria contiene:
  - Posiciones articulares (`q1` a `q6`)
  - Posición y orientación del end-effector (`x, y, z`, `roll, pitch, yaw`)
- Archivos `info.yaml` en cada subcarpeta describen el formato y metadatos.
- Trayectorias reales grabadas a ~500 Hz desde `/joint_states`.
- Datos filtrados digitalmente (Butterworth 25 Hz, orden 4) para eliminar ruido.

---

## Scripts incluidos

En la carpeta `scripts/` se encuentran los siguientes programas:

- `Generar_trayectorias_UR3e_PyBullet.py`: genera y valida trayectorias en PyBullet.
- `Ejecutar_trayectorias_UR3e.py`: ejecuta trayectorias sobre el UR3e real con MoveIt y ROS.
- `Filtrado.py`: aplica un filtro digital a las señales reales.

---

## Requisitos

- Python 3.8+
- `numpy`, `pandas`, `matplotlib`, `scipy`, `pybullet`, `roboticstoolbox`, `yaml`
- ROS Noetic y MoveIt (para ejecución real)
- Hardware: UR3e

---

## Licencia

Este repositorio está licenciado bajo los términos de la licencia MIT. Puedes consultar el archivo `LICENSE` para más información.

---

## Autor

**Marcos Sainz**  
Universidad de Deusto – Ingeniería Robótica (2025)

---

## Contacto

Para cualquier duda o sugerencia relacionada con el dataset o su uso en investigación:

> ✉️ Email: marcos.sainz@opendeusto.es

