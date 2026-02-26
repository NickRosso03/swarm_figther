"""
drone_agent.py — Agente autonomo per un singolo drone dello swarm.

Topic DDS per drone i (prefisso "drone_{i}/"):
  RICEVUTI da Godot:
    drone_{i}/X, Y, Z         : posizione [m]
    drone_{i}/VX, VY, VZ      : velocità lineare [m/s]
    drone_{i}/TX, TY, TZ      : Euler angles roll/pitch/yaw [rad]
    drone_{i}/WX, WY, WZ      : velocità angolari [rad/s]
    drone_{i}/tick            : impulso di sync ogni physics frame
    drone_{i}/connected       : 1 quando Godot è pronto

  PUBBLICATI verso Godot:
    drone_{i}/f1..f4          : forze propulsori [N]

  CONDIVISI tra agenti Python (swarm awareness):
    drone_{i}/status          : codice stato FSM (float)
    drone_{i}/sx, sy, sz      : posizione pubblicata dagli altri
    drone_{i}/fire_x,y,z      : target incendio corrente

  EVENTI INCENDIO (da Godot FireManager):
    world/fire_new            : id incendio (float, 0=nessuno)
    world/fire_x, fire_y, fire_z : posizione incendio
    world/fire_resolved       : id incendio spento
"""

import time
import math
import threading
import logging

from dds import DDS, Time
from multirotor_controller import MultirotorController
from coverage_planner import CoveragePlanner

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)

# ---------------------------------------------------------------------------
# Parametri operativi
# ---------------------------------------------------------------------------
TAKEOFF_ALT     = 8.0    # [m] quota di crociera
WAYPOINT_RADIUS = 2.0    # [m] raggio di accettazione waypoint
FIRE_RADIUS     = 2.5    # [m] raggio per iniziare soppressione
SUPPRESS_TIME   = 5.0    # [s] tempo di hover per spegnere il fuoco
N_DRONES        = 5
DDS_HOST        = '127.0.0.1'
DDS_PORT        = 4444

# ---------------------------------------------------------------------------
# Codici numerici degli stati (servono perché DDS trasporta solo float)
# ---------------------------------------------------------------------------
class StateCode:
    IDLE        = 0.0
    TAKEOFF     = 0.0
    HOVERING    = 0.0   # non disponibile durante stabilizzazione
    EXPLORING   = 1.0
    MOVING      = 2.0
    SUPPRESSING = 4.0
    RETURNING   = 3.0

class State:
    IDLE        = "IDLE"
    TAKEOFF     = "TAKEOFF"
    HOVERING    = "HOVERING"    # stabilizzazione in quota prima di esplorare
    EXPLORING   = "EXPLORING"
    MOVING      = "MOVING"
    SUPPRESSING = "SUPPRESSING"
    RETURNING   = "RETURNING"

FREE_STATES = {State.EXPLORING, State.RETURNING}


class DroneAgent:
    """
    Agente autonomo per un singolo drone.
    Ogni istanza gira nel proprio thread (vedi main.py).
    """

    def __init__(self, drone_id: int, n_drones: int = N_DRONES):
        self.id      = drone_id
        self.n       = n_drones
        self.log     = logging.getLogger(f"D{drone_id}")
        self._p      = f"drone_{drone_id}"   # prefisso topic

        self.dds     = DDS(DDS_HOST, DDS_PORT)
        self.ctrl    = MultirotorController()
        self.timer   = Time()

        # Stato fisico corrente
        self.x  = self.y  = self.z  = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.tx = self.ty = self.tz = 0.0   # roll, pitch, yaw
        self.wx = self.wy = self.wz = 0.0   # roll_rate, pitch_rate, yaw_rate

        # FSM
        self.state          = State.IDLE
        self.target_fire    = None           # [x, y, z]
        self.fire_id        = None           # float id
        self._suppress_t    = 0.0
        self._hover_start   = 0.0

        # Swarm awareness (aggiornata ogni ciclo)
        self._swarm: dict[int, dict] = {}
        self._swarm_lock = threading.Lock()

        # Piano di perlustrazione
        self.waypoints = CoveragePlanner.get_sector(
            drone_id, n_drones, area_size=150.0, altitude=TAKEOFF_ALT)
        self._wp_idx   = 0

        # Quota di decollo per il controller
        self.ctrl.set_target(z=TAKEOFF_ALT)

    # =======================================================================
    # Entry point del thread
    # =======================================================================

    def run(self):
        self._setup_dds()
        self.timer.start()

        self.log.info("In attesa di Godot (variabile 'start')...")
        self.dds.wait(f"{self._p}/connected")
        self.log.info("Godot connesso. Inizio loop di controllo.")
        self.state = State.TAKEOFF
        _dbg = 0
        _dbg_transition = 0

        while True:
            # Sincronizzazione: attendi il tick di Godot (come nel notebook del prof)
            tick = self.dds.wait(f"{self._p}/tick")
            delta_t = self.timer.elapsed()
            if delta_t <= 0:
                delta_t = 1.0 / 60.0

            # 1. Leggi sensori
            self._read_state()

            _dbg += 1

            # Log ad alta frequenza subito dopo la transizione a MOVING/EXPLORING,
            # log normale ogni 2s altrimenti — solo drone 0.
            if self.id == 0:
                _active = self.state in (State.MOVING, State.EXPLORING,
                                         State.SUPPRESSING)
                if _active and _dbg_transition == 0:
                    _dbg_transition = _dbg          # segna inizio fase attiva
                _in_crash_window = (_active and
                                    _dbg - _dbg_transition < 300)  # ~5s
                if _in_crash_window or _dbg % 120 == 0:
                    self.log.info(
                        f"[{self.state}] "
                        f"pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                        f"pitch={math.degrees(self.tx):.1f}° "
                        f"roll={math.degrees(self.ty):.1f}° | "
                        f"tgt=({self.ctrl.x_target:.1f},{self.ctrl.y_target:.1f},"
                        f"{self.ctrl.z_target:.1f}) "
                        f"vz_tgt={self.ctrl.vz_target:.2f} "
                        f"dt={delta_t*1000:.1f}ms"
                    )

            # 2. Aggiorna stato swarm
            self._update_swarm()

            # 3. FSM
            self._update_fsm(delta_t)

            # 4. Controller fisico → pubblica forze
            self._control_and_publish(delta_t)

            # 5. Pubblica il proprio stato per gli altri agenti
            self._publish_own_state()

    # =======================================================================
    # Setup DDS
    # =======================================================================

    def _setup_dds(self):
        self.dds.start()

        p = self._p
        own_vars = [
            f"{p}/X", f"{p}/Y", f"{p}/Z",
            f"{p}/VX", f"{p}/VY", f"{p}/VZ",
            f"{p}/TX", f"{p}/TY", f"{p}/TZ",
            f"{p}/WX", f"{p}/WY", f"{p}/WZ",
            f"{p}/tick", f"{p}/connected",
        ]

        swarm_vars = []
        for i in range(self.n):
            if i != self.id:
                swarm_vars += [
                    f"drone_{i}/status",
                    f"drone_{i}/sx", f"drone_{i}/sy", f"drone_{i}/sz",
                    f"drone_{i}/fire_x", f"drone_{i}/fire_y", f"drone_{i}/fire_z",
                ]

        fire_vars = [
            "world/fire_new",
            "world/fire_x", "world/fire_y", "world/fire_z",
            "world/fire_resolved",
        ]

        self.dds.subscribe(own_vars + swarm_vars + fire_vars)

    # =======================================================================
    # Lettura sensori
    # =======================================================================

    def _read_state(self):
        p = self._p
        self.x  = self.dds.read(f"{p}/X")  or 0.0
        self.y  = self.dds.read(f"{p}/Y")  or 0.0
        self.z  = self.dds.read(f"{p}/Z")  or 0.0
        self.vx = self.dds.read(f"{p}/VX") or 0.0
        self.vy = self.dds.read(f"{p}/VY") or 0.0
        self.vz = self.dds.read(f"{p}/VZ") or 0.0
        self.tx = self.dds.read(f"{p}/TX") or 0.0
        self.ty = self.dds.read(f"{p}/TY") or 0.0
        self.tz = self.dds.read(f"{p}/TZ") or 0.0
        self.wx = self.dds.read(f"{p}/WX") or 0.0
        self.wy = self.dds.read(f"{p}/WY") or 0.0
        self.wz = self.dds.read(f"{p}/WZ") or 0.0

    def _update_swarm(self):
        with self._swarm_lock:
            for i in range(self.n):
                if i == self.id:
                    continue
                self._swarm[i] = {
                    "status": self.dds.read(f"drone_{i}/status") or 0.0,
                    "pos":   [self.dds.read(f"drone_{i}/sx") or 0.0,
                              self.dds.read(f"drone_{i}/sy") or 0.0,
                              self.dds.read(f"drone_{i}/sz") or 0.0],
                    "fire":  [self.dds.read(f"drone_{i}/fire_x") or 0.0,
                              self.dds.read(f"drone_{i}/fire_y") or 0.0,
                              self.dds.read(f"drone_{i}/fire_z") or 0.0],
                }

    # =======================================================================
    # FSM
    # =======================================================================

    def _update_fsm(self, dt: float):
        if self.state == State.TAKEOFF:
            self._do_takeoff()
        elif self.state == State.HOVERING:
            self._do_hovering()
        elif self.state == State.EXPLORING:
            self._do_exploring()
            self._check_fire()
        elif self.state == State.MOVING:
            self._do_moving()
        elif self.state == State.SUPPRESSING:
            self._do_suppressing(dt)
        elif self.state == State.RETURNING:
            self._do_returning()
            self._check_fire()

    def _do_takeoff(self):
        # altitude_only=True in _control_and_publish gestisce tutto:
        # solo la quota viene controllata, XY e attitude sono bypassati.
        # Imposta z_target ma lascia x/y irrilevanti durante questo stato.
        self.ctrl.set_target(z=TAKEOFF_ALT)

        if abs(self.y - TAKEOFF_ALT) < 0.5:
            self.log.info(f"Quota raggiunta ({self.y:.1f} m). Hover di stabilizzazione.")
            self.ctrl.set_target(x=self.x, y=self.z)  # congela XY alla pos corrente
            self._hover_start = time.monotonic()
            self.state = State.HOVERING

    def _do_hovering(self):
        # Tieni il target XY aggiornato alla posizione corrente ogni frame:
        # i controller XY hanno errore ~zero, si "scaldano" senza inclinare.
        # Il PI di quota continua a girare normalmente e mantiene la quota.
        # Nessun reset — la transizione a EXPLORING è completamente liscia.
        self.ctrl.set_target(x=self.x, y=self.z)

        elapsed = time.monotonic() - self._hover_start
        if elapsed > 2.0:
            self.log.info("Hover stabile. Inizio perlustrazione.")
            self._wp_idx = self._nearest_waypoint()
            self._set_next_waypoint()
            self.state = State.EXPLORING

    def _do_exploring(self):
        wp = self.waypoints[self._wp_idx]
        if self._dist2d([self.x, self.z], wp) < WAYPOINT_RADIUS:
            self._wp_idx = (self._wp_idx + 1) % len(self.waypoints)
            self._set_next_waypoint()

    def _do_moving(self):
        if self.target_fire is None:
            self.state = State.EXPLORING
            return
        
        # -- AGGIUNTO: Interrompi se il fuoco è già stato spento da altri
        resolved_id = self.dds.read("world/fire_resolved") or 0.0
        if resolved_id == self.fire_id:
            self.log.info(f"Fuoco {self.fire_id:.0f} spento da alleati. Annullamento.")
            self.target_fire = None
            self.fire_id = None
            self.state = State.RETURNING
            return

        fx, _, fz = self.target_fire
        if self._dist2d([self.x, self.z], [fx, fz]) < FIRE_RADIUS:
            self.log.info(f"Sopra fuoco {self.fire_id:.0f}. Soppressione.")
            self._suppress_t = 0.0
            self.state = State.SUPPRESSING

    def _do_suppressing(self, dt: float):
        fx, fy, fz = self.target_fire
        self.ctrl.set_target(x=fx, y=fz)
        self._suppress_t += dt
        if self._suppress_t >= SUPPRESS_TIME:
            self.log.info(f"Fuoco {self.fire_id:.0f} spento!")
            self.dds.publish("world/fire_resolved", self.fire_id)
            self.target_fire = None
            self.fire_id     = None
            self.state       = State.RETURNING

    def _do_returning(self):
        wp = self.waypoints[self._wp_idx]

        # -- AGGIUNTO: Dobbiamo dire al controller di muoversi fisicamente verso il WP
        self.ctrl.set_target(x=wp[0], y=wp[1])

        if self._dist2d([self.x, self.z], wp) < WAYPOINT_RADIUS * 3:
            self.state = State.EXPLORING

    # =======================================================================
    # Logica swarm distribuita
    # =======================================================================

    def _check_fire(self):
        fire_id = self.dds.read("world/fire_new") or 0.0
        resolved_id = self.dds.read("world/fire_resolved") or 0.0
        if fire_id == 0.0 or fire_id == resolved_id:
            return
        if fire_id == self.fire_id:
            return

        fire_pos = [
            self.dds.read("world/fire_x") or 0.0,
            self.dds.read("world/fire_y") or 0.0,
            self.dds.read("world/fire_z") or 0.0,
        ]

        if self._should_respond(fire_id, fire_pos):
            self.log.info(f"Rispondo a fuoco {fire_id:.0f} in {fire_pos}")
            self.target_fire = fire_pos
            self.fire_id     = fire_id
            self.ctrl.set_target(x=fire_pos[0], y=fire_pos[2])
            self.state = State.MOVING

    def _should_respond(self, fire_id: float, fire_pos: list) -> bool:
        """
        Decisione distribuita: sono il candidato migliore tra i droni liberi?

        Regola:
          1. Devo essere libero (EXPLORING o RETURNING)
          2. Non ci devono essere già abbastanza droni diretti verso questo fuoco
          3. Non ci devono essere droni liberi più vicini di me
        """
        if self.state not in FREE_STATES:
            return False

        my_pos  = [self.x, self.y, self.z]
        my_dist = self._dist3d(my_pos, fire_pos)

        already_responding = 0
        closer_free        = 0

        with self._swarm_lock:
            for info in self._swarm.values():
                status = info["status"]
                # Droni già diretti verso questo fuoco
                if status == StateCode.MOVING:
                    if self._dist3d(info["fire"], fire_pos) < FIRE_RADIUS * 2:
                        already_responding += 1
                # Droni liberi più vicini di me
                if status in (StateCode.EXPLORING, StateCode.RETURNING):
                    if self._dist3d(info["pos"], fire_pos) < my_dist - 0.5:
                        closer_free += 1

        needed = 1
        if already_responding >= needed:
            return False
        if closer_free >= (needed - already_responding):
            return False
        return True

    # =======================================================================
    # Controller e pubblicazione forze
    # =======================================================================

    def _control_and_publish(self, dt: float):
        # ASSI GODOT → CONTROLLER:
        #   Godot X  → controller x  (orizzontale destra)
        #   Godot Y  → controller z  (VERTICALE = quota!)
        #   Godot Z  → controller y  (orizzontale avanti/indietro)
        #   Godot TX → roll,  WX → roll_rate
        #   Godot TY → pitch, WY → pitch_rate
        #
        # Durante TAKEOFF e HOVERING: altitude_only=True
        #   → solo spinta verticale uniforme, nessun roll/pitch
        #   → evita instabilità da termini P attitude durante la salita
        altitude_only = self.state == State.TAKEOFF

        f1, f2, f3, f4 = self.ctrl.evaluate(
            delta_t       = dt,
            z=self.y,  vz=self.vy,   # Godot Y è la quota
            x=self.x,  vx=self.vx,
            y=self.z,  vy=self.vz,   # Godot Z è il secondo asse orizzontale
            roll=self.tz,  roll_rate=self.wz,      # <-- TZ è il ROLL
            pitch=self.tx, pitch_rate=self.wx,     # <-- TX è il PITCH
            altitude_only = altitude_only,
        )
        p = self._p
        self.dds.publish(f"{p}/f1", f1)
        self.dds.publish(f"{p}/f2", f2)
        self.dds.publish(f"{p}/f3", f3)
        self.dds.publish(f"{p}/f4", f4)

    def _publish_own_state(self):
        p = self._p
        sc = {
            State.IDLE:        StateCode.IDLE,
            State.TAKEOFF:     StateCode.TAKEOFF,
            State.HOVERING:    StateCode.HOVERING,
            State.EXPLORING:   StateCode.EXPLORING,
            State.MOVING:      StateCode.MOVING,
            State.SUPPRESSING: StateCode.SUPPRESSING,
            State.RETURNING:   StateCode.RETURNING,
        }.get(self.state, 0.0)
        self.dds.publish(f"{p}/status", sc)
        self.dds.publish(f"{p}/sx", self.x)
        self.dds.publish(f"{p}/sy", self.y)
        self.dds.publish(f"{p}/sz", self.z)
        fx, fy, fz = self.target_fire if self.target_fire else (0.0, 0.0, 0.0)
        self.dds.publish(f"{p}/fire_x", fx)
        self.dds.publish(f"{p}/fire_y", fy)
        self.dds.publish(f"{p}/fire_z", fz)

        # --- NUOVE RIGHE PER IL DEBUG ---
        # Il target y del controller equivale all'asse Z di Godot
        self.dds.publish(f"{p}/tgt_x", self.ctrl.x_target)
        self.dds.publish(f"{p}/tgt_z", self.ctrl.y_target)

    # =======================================================================
    # Utility
    # =======================================================================

    def _nearest_waypoint(self) -> int:
        """Restituisce l'indice del waypoint più vicino alla posizione corrente."""
        best_idx  = 0
        best_dist = float('inf')
        for i, wp in enumerate(self.waypoints):
            d = self._dist2d([self.x, self.z], wp)
            if d < best_dist:
                best_dist = d
                best_idx  = i
        self.log.info(f"Waypoint più vicino: #{best_idx} {self.waypoints[best_idx]} (dist={best_dist:.1f}m)")
        return best_idx

    def _set_next_waypoint(self):
        wp = self.waypoints[self._wp_idx]
        # wp[0] = X Godot, wp[1] = Z Godot (piano orizzontale)
        self.ctrl.set_target(x=wp[0], y=wp[1])

    @staticmethod
    def _dist2d(a, b) -> float:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    @staticmethod
    def _dist3d(a, b) -> float:
        return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))