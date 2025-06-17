# ======================================================
# CÓDIGO: Ejecutar_Trayectorias_UR3e
# ======================================================
#!/usr/bin/env python3
import rospy
import csv
import glob
import os
import re
import datetime
import time
from copy import deepcopy

from moveit_commander import RobotCommander, MoveGroupCommander, RobotTrajectory, PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.msg import SafetyMode
from geometry_msgs.msg import PoseStamped

# ======================================================
# Clase Recorder500Hz: registro de estados articulares
# ======================================================
class Recorder500Hz:
    """
    Registra mensajes /joint_states a ~500 Hz.
    Acumula en memoria cada muestra hasta un tiempo dado y salva en CSV.

    Métodos:
        __init__(self, csv_path: str, duration: float)
        cb_joints(self, msg: JointState)
        save_csv(self) -> None
    """
    def __init__(self, csv_path: str, duration: float):
        """
        Inicializa el recorder.

        Args:
            csv_path (str): Ruta de salida del CSV con datos.
            duration (float): Duración máxima de grabación en segundos.
        """
        self.csv_path = csv_path
        self.duration = duration
        self.start_time = None
        self.registros = []
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.cb_joints)

    def cb_joints(self, msg: JointState):
        """
        Callback ROS para /joint_states.

        Args:
            msg (JointState): Mensaje con posiciones, velocidades y esfuerzos.
        """
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now

        if now - self.start_time <= self.duration:
            pos = list(msg.position[:6])
            vel = list(msg.velocity[:6])
            eff = list(msg.effort[:6])
            row = [now] + pos + vel + eff
            self.registros.append(row)
        else:
            # Duración cumplida: detener la suscripción y avisar a ROS para apagar el nodo si hace falta
            self.subscriber.unregister()
            # No hacemos signal_shutdown aquí porque dejamos que el script principal controle el flujo.

    def save_csv(self):
        """
        Guarda los registros acumulados en CSV.

        Returns:
            None
        """
        header = ['timestamp'] \
                 + [f'q{i+1}' for i in range(6)] \
                 + [f'qd{i+1}' for i in range(6)] \
                 + [f'tau{i+1}' for i in range(6)]
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for row in self.registros:
                writer.writerow(row)
        rospy.loginfo(f"[Recorder500Hz] Guardadas {len(self.registros)} muestras en {self.csv_path}")

# ======================================================
# Definición de variables y funciones auxiliares
# ======================================================

joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

# Variables globales
joints = JointState()  #Último estado de articulaciones
safety_mode_code = 1  #Código de modo de seguridad actual
SAFETY_MODE_LABELS = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY"
}

home_joints = [0, -1.57, 1.57, 0, 1.57, 0]

INTERPOLADORES_VALIDOS = {
    "jtraj": "JTRAJ",
    "quintic": "QUINTIC",
    "quintic_func": "QUINTIC_FUNC",
    "trapezoidal": "TRAPEZOIDAL",
    "trapezoidal_func": "TRAPEZOIDAL_FUNC"
}

def solicitar_interpolador() -> str:
    """
    Solicita al usuario elegir interpolador.

    Args:
        None
    Returns:
        seleccion (str): Key válida de INTERPOLADORES_VALIDOS.
    """
    print("Interpoladores disponibles:")
    for key in INTERPOLADORES_VALIDOS:
        print(f" - {key}")
    while True:
        sel = input("Interpolador: ").strip().lower()
        if sel in INTERPOLADORES_VALIDOS:
            return sel
        print("No válido, inténtalo de nuevo.")


def solicitar_objetivo_validas() -> int:
    """
    Solicita número de trayectorias válidas a ejecutar.

    Returns:
        int (>0)
    """
    while True:
        val = input("Número de trayectorias válidas: ").strip()
        if val.isdigit() and int(val) > 0:
            return int(val)
        print("Debe ser un entero positivo.")


def cb_safety_mode(msg: SafetyMode) -> None:
    """
    Callback para actualizar modo de seguridad.

    Args:
        msg (SafetyMode): Código de modo.
    """
    global safety_mode_code
    safety_mode_code = msg.mode


def cb_joints_global(msg: JointState) -> None:
    """
    Callback para actualizar estado global de articulaciones.

    Args:
        msg (JointState)
    """
    global joints
    joints = deepcopy(msg)


def extraer_numero_trayectoria(fpath: str) -> int:
    """
    Extrae primer número en nombre de archivo.

    Args:
        fpath (str): Ruta o nombre.
    Returns:
        int o -1
    """
    name = os.path.basename(fpath)
    m = re.search(r"(\d+)", name)
    return int(m.group()) if m else -1


def read_trajectory_from_csv(csv_path: str, joint_names: list) -> list:
    """
    Lee posiciones articulares de CSV.

    Args:
        csv_path: ruta archivo con columnas joint_names.
        joint_names: lista de nombres.
    Returns:
        List[List[float]] posiciones.
    """
    traj = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            traj.append([float(row[j]) for j in joint_names])
    return traj


def create_trajectory_msg(joint_names: list, configs: list) -> RobotTrajectory:
    """
    Genera RobotTrajectory desde configuraciones.

    Args:
        joint_names: lista de nombres.
        configs: lista de listas de floats.
    Returns:
        RobotTrajectory con puntos.
    """
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = joint_names
    for cfg in configs:
        p = JointTrajectoryPoint()
        p.positions = cfg
        traj.joint_trajectory.points.append(p)
    return traj


def guardar_trayectoria_csv(path: str, registros: list) -> None:
    """
    Guarda registros (sim o real) en CSV con cabecera.

    Args:
        path: ruta salida
        registros: lista de listas
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        header = ['timestamp'] + [f'q_{j}' for j in joint_names] + [f'qd_{j}' for j in joint_names] + [f'tau_{j}' for j in joint_names]
        w.writerow(header)
        w.writerows(registros)


def mover_a_home(move_group: MoveGroupCommander, home_joints: list) -> bool:
    """
    Envía robot a posición home.

    Args:
        move_group: MoveIt commander
        home_joints: lista de 6 floats
    Returns:
        True si ok
    """
    move_group.go(home_joints, wait=True)
    move_group.stop()
    return True

def registrar_error_trayectoria(nombre_trayectoria: str, tipo_error: str, detalle: str,
                                 safety_mode_code: int, errors_log_path: str) -> None:
    """
    Registra un error ocurrido durante la ejecución de una trayectoria.

    Args:
        nombre_trayectoria (str): Identificador o nombre del archivo de trayectoria.
        tipo_error (str): Categoría del error (p.ej. "planificacion", "seguridad").
        detalle (str): Descripción específica del fallo.
        safety_mode_code (int): Código de modo de seguridad al momento del error.
        errors_log_path (str): Ruta del CSV de log de errores.
    Returns:
        None (escribe una fila en el CSV de errores).
    """
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    write_header = not os.path.exists(errors_log_path)
    with open(errors_log_path, "a", newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp", "trayectoria", "tipo_error", "detalle", "codigo_safety_mode"])
        writer.writerow([timestamp, nombre_trayectoria, tipo_error, detalle, safety_mode_code])


def cargar_trayectorias_fallidas(errors_log: str) -> set:
    """
    Carga el conjunto de trayectorias que han fallado según el log de errores.

    Args:
        errors_log (str): Ruta del CSV de errores.
    Returns:
        set[str]: Nombres de trayectorias que han registrado al menos un error.
    """
    if not os.path.exists(errors_log):
        return set()
    with open(errors_log, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        return set(row['trayectoria'] for row in reader)


def cargar_trayectorias_ok(results_ok_dir: str) -> set:
    """
    Carga el conjunto de trayectorias que se completaron correctamente.

    Args:
        results_ok_dir (str): Directorio donde se guardan resultados exitosos (_result.csv).
    Returns:
        set[str]: Nombre base de archivos de trayectoria sin sufijo _result.csv.
    """
    if not os.path.exists(results_ok_dir):
        return set()
    return set(
        fname.replace("_result.csv", ".csv")
        for fname in os.listdir(results_ok_dir)
        if fname.endswith("_result.csv")
    )


def esperar_modo_normal(timeout: float = 10.0, verificacion_continua: float = 2.0) -> bool:
    """
    Espera hasta que el robot permanezca en modo NORMAL durante un periodo continuo.

    Args:
        timeout (float): Tiempo máximo de espera total en segundos.
        verificacion_continua (float): Duración mínima en modo NORMAL para considerarlo estable.
    Returns:
        bool: True si alcanza modo NORMAL estable dentro del timeout, False si expira.
    """
    global safety_mode_code
    tiempo_inicio = time.time()
    tiempo_normal = None
    while time.time() - tiempo_inicio < timeout:
        if safety_mode_code == 1:
            if tiempo_normal is None:
                tiempo_normal = time.time()
            elif time.time() - tiempo_normal >= verificacion_continua:
                return True
        else:
            tiempo_normal = None
        time.sleep(0.05)
    return False

# ======================================================
# Función main(): configuración, reanudación y ejecución
# ======================================================
def main():
    """
    Función principal para:
     1) Solicitar interpolador y número objetivo de trayectorias.
     2) Definir rutas dinámicas (datos, resultados, logs).
     3) Inicializar ROS, MoveIt! y escena de planificación.
     4) Listar trayectorias disponibles y gestionar reanudación.
     5) Iterar y ejecutar cada trayectoria con validación completa:
        a) Verificar modo NORMAL estable.
        b) Leer CSV simulada.
        c) Mover a home.
        d) Planificar movimiento al primer punto.
        e) Retrasar y planificar trayectoria completa.
        f) Preparar recorder y ejecutar sin bloqueo.
        g) Monitorear fin de movimiento.
        h) Guardar grabación y validar destino final.
        i) Registrar errores o confirmar éxito.
    """
    global safety_mode_code, joints

    # 1) Solicitar parámetros al usuario
    interpolador = solicitar_interpolador()
    nombre_interp_upper = INTERPOLADORES_VALIDOS[interpolador]
    objetivo_validas = solicitar_objetivo_validas()

    # 2) Definir rutas de archivos y carpetas
    TRAYECTORIAS_DIR = os.path.join("data", f"trayectorias_completas_UR3e__{nombre_interp_upper}")
    RESULTS_OK_DIR = os.path.join("trayectorias_ejecutadas", interpolador)
    ERRORS_LOG = os.path.join("logs", f"registro_errores_trayectorias_{interpolador}.csv")

    os.makedirs(RESULTS_OK_DIR, exist_ok=True)
    os.makedirs(os.path.dirname(ERRORS_LOG), exist_ok=True)

    # 3) Inicializar nodo ROS y suscriptores
    rospy.init_node("ejecutar_trayectorias_batch", anonymous=False)
    rospy.Subscriber("/joint_states", JointState, cb_joints_global)
    rospy.Subscriber("/robot/ur_hardware_interface/safety_mode", SafetyMode, cb_safety_mode)

    # 4) Configurar MoveIt! y escena de planificación
    move_group = MoveGroupCommander("robot")
    robot = RobotCommander()
    joint_names_robot = move_group.get_active_joints()
    assert joint_names == joint_names_robot, "Nombres de articulaciones incorrectos."  
    scene = PlanningSceneInterface()
    rospy.sleep(2)

    #Añadir mesa 
    pose_mesa = PoseStamped()
    pose_mesa.header.frame_id = robot.get_planning_frame()
    pose_mesa.pose.position.x = 0; pose_mesa.pose.position.y = 0; pose_mesa.pose.position.z = -0.05
    scene.add_box("mesa", pose_mesa, size=(1.5, 0.8, 0.1))
    rospy.sleep(2)

    # 5) Listar y filtrar trayectorias
    trayectorias_files = sorted(
        glob.glob(os.path.join(TRAYECTORIAS_DIR, "trayectoria_*.csv")),
        key=extraer_numero_trayectoria
    )
    print(f"Total trayectorias disponibles: {len(trayectorias_files)}")
    if not trayectorias_files:
        print(f"[ERROR] No se han encontrado trayectorias en {TRAYECTORIAS_DIR}.")
        return

    trayectorias_ok = cargar_trayectorias_ok(RESULTS_OK_DIR)
    trayectorias_fallidas = cargar_trayectorias_fallidas(ERRORS_LOG)
    contador_validas = len(trayectorias_ok)
    #Seleccionar pendientes
    trayectorias_pendientes = [f for f in trayectorias_files
                                if os.path.basename(f) not in trayectorias_ok
                                and os.path.basename(f) not in trayectorias_fallidas]
    print(f"OK: {len(trayectorias_ok)}, Fallidas: {len(trayectorias_fallidas)}, Pendientes: {len(trayectorias_pendientes)}")
    if not trayectorias_pendientes:
        print("No hay trayectorias pendientes. Proceso finalizado.")
        return

    #Opción de reanudar o reiniciar todo
    while True:
        resp = input("Continuar con pendientes (s) o reiniciar todo (n)? [s/n]: ").strip().lower()
        if resp in ("s","n"): break
    trayectorias_a_ejecutar = trayectorias_pendientes if resp=="s" else trayectorias_files

    # 6) Parámetros de tolerancia y control
    EPSILON_POS, EPSILON_VEL = 1e-3, 1e-3
    STOPPED_SAMPLES = 30
    MAX_TIEMPO_REGISTRO = 60.0
    errores_consecutivos = 0

    # 7) Bucle principal de ejecución detallado
    for traj_path in trayectorias_a_ejecutar:
        nombre_tray = os.path.basename(traj_path)

        # a) Verificar modo NORMAL estable
        if not esperar_modo_normal(timeout=10.0, verificacion_continua=2.0):
            detalle = f"Safety code={safety_mode_code}, label={SAFETY_MODE_LABELS.get(safety_mode_code,'UNKNOWN')}"
            print(f"ATENCIÓN {nombre_tray}: modo seguridad inestable: {detalle}")
            registrar_error_trayectoria(nombre_tray, "SAFETY_MODE_NOT_NORMAL_SOSTENIDO", detalle, safety_mode_code, ERRORS_LOG)
            break

        rospy.loginfo(f"== Ejecutando {nombre_tray} ==")

        # b) Leer trayectoria simulada desde CSV
        try:
            joint_trajectory = read_trajectory_from_csv(traj_path, joint_names)
        except Exception as e:
            detalle = f"Error leyendo CSV: {e}"
            print(f"{nombre_tray}: {detalle}")
            registrar_error_trayectoria(nombre_tray, "ERROR_LECTURA_CSV", detalle, safety_mode_code, ERRORS_LOG)
            mover_a_home(move_group, home_joints)
            errores_consecutivos += 1
            continue

        # c) Mover robot a home antes de iniciar
        if not mover_a_home(move_group, home_joints):
            detalle = "Fallo al mover a home"
            print(f"{nombre_tray}: {detalle}")
            registrar_error_trayectoria(nombre_tray, "HOME_CONTROL_FAILED", detalle, safety_mode_code, ERRORS_LOG)
            errores_consecutivos += 1
            if errores_consecutivos >= 20:
                print("Demasiados errores consecutivos. Finalizando.")
                break
            continue
        rospy.sleep(1)  #Pequeña pausa para estabilizar

        # d) Movilizar al primer punto de la trayectoria
        try:
            move_group.set_joint_value_target(joint_trajectory[0])
            success = move_group.go(wait=True)
            if not success:
                raise RuntimeError("Planificación/execution al primer punto fallida")
        except Exception as e:
            detalle = str(e)
            print(f"{nombre_tray}: {detalle}")
            registrar_error_trayectoria(nombre_tray, "ERROR_MOVER_PRIMER_PUNTO", detalle, safety_mode_code, ERRORS_LOG)
            mover_a_home(move_group, home_joints)
            errores_consecutivos += 1
            continue

        # e) Retime y planificar trayectoria completa
        try:
            traj_msg = create_trajectory_msg(joint_names, joint_trajectory)
            current_state = robot.get_current_state()
            timed = move_group.retime_trajectory(
                ref_state_in=current_state,
                traj_in=traj_msg,
                velocity_scaling_factor=0.1,
                acceleration_scaling_factor=0.1,
                algorithm="iterative_time_parameterization"
            )
            if not (timed and timed.joint_trajectory.points):
                raise ValueError("No se encontró plan de trayectoria completa")
        except Exception as e:
            detalle = str(e)
            print(f"{nombre_tray}: {detalle}")
            registrar_error_trayectoria(nombre_tray, "ERROR_PLANIFICACION", detalle, safety_mode_code, ERRORS_LOG)
            mover_a_home(move_group, home_joints)
            errores_consecutivos += 1
            continue

        # f) Preparar grabación: duración = tiempo final + margen
        T_final = timed.joint_trajectory.points[-1].time_from_start.to_sec()
        duracion_registro = T_final + 2.0
        csv_real = os.path.join(RESULTS_OK_DIR, nombre_tray.replace(".csv", "_result.csv"))
        recorder = Recorder500Hz(csv_real, duracion_registro)
        rospy.loginfo(f"[Recorder] Grabando {nombre_tray} durante ~{duracion_registro:.2f}s")

        # g) Ejecutar trayectoria sin bloqueo
        try:
            move_group.execute(timed, wait=False)
        except Exception as e:
            detalle = str(e)
            print(f"{nombre_tray}: {detalle}")
            registrar_error_trayectoria(nombre_tray, "ERROR_EJECUCION", detalle, safety_mode_code, ERRORS_LOG)
            recorder.subscriber.unregister()
            recorder.save_csv()
            mover_a_home(move_group, home_joints)
            errores_consecutivos += 1
            continue

        # h) Monitorear el movimiento hasta completar
        started = False
        stopped_count = 0
        last_point = joint_trajectory[-1]
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - t0 > MAX_TIEMPO_REGISTRO:
                registrar_error_trayectoria(nombre_tray, "TIMEOUT", "Tiempo excedido", safety_mode_code, ERRORS_LOG)
                break
            if not joints.position:
                rospy.sleep(0.001); continue
            pos_act = list(joints.position)
            vel_act = list(joints.velocity)
            if any(abs(pa-pi)>EPSILON_POS for pa,pi in zip(pos_act, joint_trajectory[0])):
                started = True
            if started:
                en_obj = all(abs(pa-pf)<EPSILON_POS for pa,pf in zip(pos_act, last_point))
                parada = all(abs(v)<EPSILON_VEL for v in vel_act)
                stopped_count = stopped_count+1 if en_obj and parada else 0
                if stopped_count>=STOPPED_SAMPLES:
                    break
            if safety_mode_code!=1:
                registrar_error_trayectoria(nombre_tray, "SAFETY_STOP", "Parada de seguridad", safety_mode_code, ERRORS_LOG)
                mover_a_home(move_group, home_joints)
                break
            rospy.sleep(0.001)

        # i) Finalizar grabación y validar resultados
        recorder.subscriber.unregister()
        recorder.save_csv()
        #Verificar destino final
        if started:
            if recorder.registros:
                last_saved = recorder.registros[-1][1:7]
                if not all(abs(ls-lp)<EPSILON_POS for ls,lp in zip(last_saved, last_point)):
                    registrar_error_trayectoria(nombre_tray, "NO_DESTINATION", "No alcanzó fin", safety_mode_code, ERRORS_LOG)
                    mover_a_home(move_group, home_joints)
                    errores_consecutivos += 1
                    continue
            else:
                registrar_error_trayectoria(nombre_tray, "SIN_MUESTRAS", "Sin samples", safety_mode_code, ERRORS_LOG)
                mover_a_home(move_group, home_joints)
                errores_consecutivos += 1
                continue
        #Registrar éxito
        rospy.loginfo(f"Trayectoria {nombre_tray} completada con éxito.")
        contador_validas += 1
        if contador_validas>=objetivo_validas:
            rospy.loginfo(f"Objetivo alcanzado: {contador_validas} trayectorias válidas.")
            break

    rospy.loginfo("== Procesamiento finalizado. ==")

if __name__ == "__main__":
    main()