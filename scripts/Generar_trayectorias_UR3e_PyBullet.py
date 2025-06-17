# ======================================================
# CÓDIGO: Generar_Trayectorias_UR3e_PyBullet
# ======================================================

import pandas as pd
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import os
from roboticstoolbox import quintic, jtraj
from roboticstoolbox.tools.trajectory import quintic_func, trapezoidal_func, trapezoidal  
import pickle
import yaml

# ======================================================
# FUNCIÓN: fijar_pinza_en_cada_paso
# ======================================================
def fijar_pinza_en_cada_paso(robot_id):
    """
    Fija las articulaciones de la pinza del robot UR3e a una posición neutral.
    Esto es útil para evitar que las posiciones de la pinza interfieran con las validaciones de colisión.

    Parámetros:
        robot_id (int): ID del robot cargado en PyBullet.
    """
    posiciones_iniciales_pinza = {
        'finger_joint': 0.0,
        'left_inner_finger_joint': 0.0,
        'left_inner_knuckle_joint': 0.0,
        'right_outer_knuckle_joint': 0.0,
        'right_inner_finger_joint': 0.0,
        'right_inner_knuckle_joint': 0.0
    }
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if joint_name in posiciones_iniciales_pinza:
            p.resetJointState(robot_id, i, posiciones_iniciales_pinza[joint_name])

# ======================================================
# FUNCIÓN: get_link_names
# ======================================================
def get_link_names(robot_id):
    """
    Devuelve un diccionario que asocia los índices de los enlaces del robot con sus nombres.
    Esto facilita la identificación de enlaces durante la detección de colisiones.

    Parámetros:
        robot_id (int): ID del robot cargado en PyBullet.

    Retorna:
        dict: Diccionario {índice: nombre} de enlaces.
    """
    link_names = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        link_index = joint_info[0]
        link_name = joint_info[12].decode("utf-8")
        link_names[link_index] = link_name
    link_names[-1] = "base_link"
    return link_names

# ======================================================
# FUNCIÓN AUXILIAR: generar_yaml_metadata
# ======================================================
def generar_yaml_metadata(carpeta, nombre_interp, num_trayectorias):
    """
    Genera un archivo info.yaml con los metadatos de la carpeta de trayectorias.

    Parámetros:
        carpeta (str): Ruta de la carpeta donde se guardará el archivo YAML.
        nombre_interp (str): Nombre del interpolador usado.
        num_trayectorias (int): Número total de trayectorias válidas generadas.
    """
    metadata = {
        'interpolador': nombre_interp,
        'formato': 'CSV',
        'trayectorias_generadas': num_trayectorias,
        'puntos_por_trayectoria': 150,
        'joints': [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ],
        'tipo_muestreo': 'aleatorio_uniforme',
        'descripcion': 'Trayectorias generadas en simulacion para el UR3e, validadas cinematicamente y sin colisiones.'
    }
    with open(os.path.join(carpeta, 'info.yaml'), 'w') as file:
        yaml.dump(metadata, file, default_flow_style=False, sort_keys=False)

# ======================================================
# FUNCIÓN: is_configuration_valid
# ======================================================
def is_configuration_valid(robot_id, joint_angles, link_names):
    """
    Verifica si una configuración articular es válida, es decir, si no provoca colisiones importantes
    con otros enlaces del robot ni con el suelo. Se ignoran ciertas colisiones internas previstas.

    Parámetros:
        robot_id (int): ID del robot cargado en PyBullet.
        joint_angles (list of float): Lista de valores articulares en radianes.
        link_names (dict): Diccionario {índice: nombre} de enlaces.

    Retorna:
        bool: True si no hay colisiones relevantes, False en caso contrario.
    """
    joint_names_brazo = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    indices_brazo = []
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        if name in joint_names_brazo:
            indices_brazo.append(i)

    for i, idx in enumerate(indices_brazo):
        p.resetJointState(robot_id, idx, joint_angles[i])

    p.stepSimulation()
    p.performCollisionDetection()
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)

    ignored_self_collisions = {
        frozenset({'left_inner_finger', 'left_inner_knuckle'}),
        frozenset({'left_outer_knuckle', 'left_inner_knuckle'}),
        frozenset({'left_inner_finger', 'left_outer_knuckle'}),
        frozenset({'right_inner_finger', 'right_inner_knuckle'}),
        frozenset({'right_outer_knuckle', 'right_inner_knuckle'}),
        frozenset({'left_inner_knuckle', 'right_inner_knuckle'}),
        frozenset({'left_inner_finger', 'right_inner_finger'}),
        frozenset({'left_inner_finger', 'right_inner_knuckle'}),
        frozenset({'left_inner_finger', 'right_outer_knuckle'}),
        frozenset({'left_inner_knuckle', 'right_inner_finger'}),
        frozenset({'left_inner_knuckle', 'right_outer_knuckle'}),
        frozenset({'left_outer_knuckle', 'right_inner_finger'}),
        frozenset({'left_outer_knuckle', 'right_inner_knuckle'}),
        frozenset({'left_outer_knuckle', 'right_outer_knuckle'}),
        frozenset({'right_inner_finger', 'right_outer_knuckle'}),
        frozenset({'onrobot_rg2_base_link', 'right_inner_finger'}),
        frozenset({'onrobot_rg2_base_link', 'left_inner_finger'}),
        frozenset({'onrobot_rg2_base_link', 'left_outer_knuckle'}),
        frozenset({'onrobot_rg2_base_link', 'left_inner_knuckle'}),
        frozenset({'onrobot_rg2_base_link', 'right_outer_knuckle'}),
        frozenset({'onrobot_rg2_base_link', 'right_inner_knuckle'}),
        frozenset({'wrist_3_link', 'onrobot_rg2_base_link'})
    }

    for contact in contacts:
        linkA = contact[3]
        linkB = contact[4]
        nameA = link_names.get(linkA)
        nameB = link_names.get(linkB)
        if linkA != linkB:
            pair = frozenset({nameA, nameB})
            if pair not in ignored_self_collisions:
                return False

    contacts_with_ground = p.getClosestPoints(bodyA=robot_id, bodyB=0, distance=0.0001)
    for contact in contacts_with_ground:
        contact_distance = contact[8]
        if contact_distance < 0.0001:
            return False
        
    return True

# ======================================================
# FUNCIÓN: generar_trayectorias_con_multiples_interpoladores
# ======================================================
def generar_trayectorias_con_multiples_interpoladores(cantidad_deseada, robot_id):
    """
    Genera trayectorias de movimiento entre configuraciones articulares aleatorias para el brazo UR3e
    empleando diferentes métodos de interpolación, verificando su validez física y almacenando
    únicamente las trayectorias sin colisiones.

    Parámetros:
        cantidad_deseada (int): Número de trayectorias válidas a generar por interpolador.
        robot_id (int): ID del robot URDF cargado en el entorno de simulación de PyBullet.
    """

    #Configuración inicial de la gravedad y obtención de nombres de enlaces
    p.setGravity(0, 0, -9.8)
    link_names = get_link_names(robot_id)

    #Lista de nombres de métodos de interpolación a aplicar
    interpoladores = ["JTRAJ", "QUINTIC", "QUINTIC_FUNC", "TRAPEZOIDAL", "TRAPEZOIDAL_FUNC"]

    cont_trayectoria = 0  #Contador total de intentos de generación
    carpetas = {}         #Diccionario para almacenar rutas por interpolador
    contadores_validos = {}  #Contadores de trayectorias válidas por interpolador

    #Crear carpetas de salida y cargar estado si se reanuda
    for nombre in interpoladores:
        carpeta = f"trayectorias_completas_UR3e_{nombre}"
        os.makedirs(carpeta, exist_ok=True)
        carpetas[nombre] = carpeta

        estado_path = os.path.join(carpeta, "estado.pkl")
        print(f"Buscando estado en {estado_path}...")

        if os.path.exists(estado_path):
            with open(estado_path, "rb") as f:
                estado = pickle.load(f)
                contadores_validos[nombre] = estado["contador"]
                np.random.set_state(estado["semilla"])
                print(f"Reanudando {nombre} desde la trayectoria {contadores_validos[nombre] + 1}...")
        else:
            contadores_validos[nombre] = 0
            print(f"Generando desde cero para {nombre}...")

    #Rango permitido por articulación 
    rangos_articulaciones = {
        'Base': (-2 * np.pi, 2 * np.pi),
        'Hombro': (-2.8, 2 * np.pi),
        'Codo': (-2.6, 2.6),
        'Muñeca 1': (-2 * np.pi, 2 * np.pi),
        'Muñeca 2': (-2 * np.pi, 2 * np.pi),
        'Muñeca 3': (-2 * np.pi, 2 * np.pi)
    }

    #Bucle principal: se ejecuta hasta obtener la cantidad requerida de trayectorias válidas por interpolador
    while any(contadores_validos[n] < cantidad_deseada for n in interpoladores):
        #Generar configuraciones articulares aleatorias (inicio y fin)
        inicio = np.array([np.random.uniform(*rangos_articulaciones[nombre]) for nombre in rangos_articulaciones])
        fin = np.array([np.random.uniform(*rangos_articulaciones[nombre]) for nombre in rangos_articulaciones])
        t = np.linspace(0, 1, 150)  #Tiempo normalizado a 150 puntos

        cont_trayectoria += 1
        print(f"\nTrayectoria base generada #{cont_trayectoria}")

        #Probar cada interpolador
        for nombre_interp in interpoladores:
            if contadores_validos[nombre_interp] >= cantidad_deseada:
                continue

            # === Interpolación ===
            if nombre_interp == "JTRAJ":
                traj = jtraj(inicio, fin, t)
                trayectorias = traj.q
            elif nombre_interp == "QUINTIC":
                traj_list = [quintic(inicio[i], fin[i], t) for i in range(len(inicio))]
                trayectorias = np.column_stack([traj.q for traj in traj_list])
            elif nombre_interp == "QUINTIC_FUNC":
                traj_func_list = [quintic_func(inicio[i], fin[i], t[-1]) for i in range(len(inicio))]
                trayectorias = np.column_stack([traj_func(t)[0] for traj_func in traj_func_list])
            elif nombre_interp == "TRAPEZOIDAL":
                traj_trap_list = []
                for i in range(len(inicio)):
                    try:
                        traj = trapezoidal(inicio[i], fin[i], t)
                        traj_trap_list.append(traj.q)
                    except ValueError as e:
                        print(f"Error en la articulación {i+1}: {e}")
                        traj_trap_list.append(np.zeros_like(t))
                trayectorias = np.column_stack(traj_trap_list)
            elif nombre_interp == "TRAPEZOIDAL_FUNC":
                tf = t[-1]
                traj_func_list = [trapezoidal_func(inicio[i], fin[i], tf) for i in range(len(inicio))]
                trayectorias = np.column_stack([traj_func(t)[0] for traj_func in traj_func_list])

            trayectoria_valida = True
            trayectoria_actual = []

            #Verificación de validez para cada punto de la trayectoria
            for j, punto in enumerate(trayectorias):
                fijar_pinza_en_cada_paso(robot_id)
                if not is_configuration_valid(robot_id, punto, link_names):
                    trayectoria_valida = False
                    break

                #Aplicar configuración articular
                joint_names_brazo = [
                    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                ]
                indices_brazo = [i for i in range(p.getNumJoints(robot_id))
                                 if p.getJointInfo(robot_id, i)[1].decode('utf-8') in joint_names_brazo]

                for i, idx in enumerate(indices_brazo):
                    p.resetJointState(robot_id, idx, punto[i])

                #Obtener posición y orientación del end-effector (Tool0)
                tool0_index = 7
                link_state = p.getLinkState(robot_id, tool0_index)
                position = link_state[0]
                orientation_rpy = p.getEulerFromQuaternion(link_state[1])

                pose_info = {
                    'Punto': j,
                    **{name: punto[i] for i, name in enumerate(joint_names_brazo)},
                    'Posicion_X': position[0],
                    'Posicion_Y': position[1],
                    'Posicion_Z': position[2],
                    'Roll': orientation_rpy[0],
                    'Pitch': orientation_rpy[1],
                    'Yaw': orientation_rpy[2]
                }
                trayectoria_actual.append(pose_info)

            #Si es válida, guardar CSV y estado
            if trayectoria_valida:
                contadores_validos[nombre_interp] += 1
                carpeta = carpetas[nombre_interp]
                df_trayectoria = pd.DataFrame(trayectoria_actual)
                nombre_archivo = os.path.join(
                    carpeta, f"trayectoria_{contadores_validos[nombre_interp]:03d}.csv")
                df_trayectoria.to_csv(nombre_archivo, index=False)
                print(f"{nombre_interp} guardada como válida ({contadores_validos[nombre_interp]})")

                estado = {
                    "semilla": np.random.get_state(),
                    "contador": contadores_validos[nombre_interp]
                }
                with open(os.path.join(carpeta, "estado.pkl"), "wb") as f:
                    pickle.dump(estado, f)
            else:
                print(f"{nombre_interp} descartada por colisión.")


    for nombre_interp in interpoladores:
        carpeta = carpetas[nombre_interp]
        generar_yaml_metadata(carpeta, nombre_interp, contadores_validos[nombre_interp])

    print("\nTodas las trayectorias válidas generadas y sus carpetas cuentan con metadatos info.yaml.")


if __name__ == "__main__":

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")  

    urdf_path = os.path.join("robot_description_files", "ur3e_rg2", "ur3e_rg2_205.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.01], useFixedBase=True,
                      flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

    cantidad_deseada=5000
    print("Simulación iniciada.")
    generar_trayectorias_con_multiples_interpoladores(cantidad_deseada, robot_id)

    while True:
        pass       
