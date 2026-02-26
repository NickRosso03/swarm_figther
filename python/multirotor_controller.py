"""
multirotor_controller.py

Schema fedele agli appunti del prof (foto allegate), con feedforward hover.

═══════════════════════════════════════════════════════════════════════
SCHEMA A BLOCCHI (dal quaderno del prof, foto 1-2)
═══════════════════════════════════════════════════════════════════════

  LOOP QUOTA
  ┌──────────────────────────────────────────────────────────┐
  │  z_tgt ──►[Σ]──► P ──────► vz_tgt                       │
  │            ▲   (kp=2,       (sat ±2 m/s)                 │
  │           z        sat 2)                                │
  │                       vz_tgt ──►[Σ]──► PID ──► f_corr   │
  │                                  ▲   (kp=5,              │
  │                                 vz    ki=2,              │
  │                                       sat ±5)            │
  │                       f_base = HOVER_FF + f_corr         │
  └──────────────────────────────────────────────────────────┘

  LOOP POSIZIONE XY  (due catene speculari, schema foto 1)
  ┌──────────────────────────────────────────────────────────┐
  │  x_tgt ──►[Σ]──► P ──────► vx_tgt                       │
  │            ▲   (kp=0.5,     (sat ±1.5 m/s)              │
  │           x        sat 1.5)                              │
  │                       vx_tgt ──►[Σ]──► P ──► pitch_tgt  │
  │                                  ▲  (kp=0.4,            │
  │                                 vx   sat ±8°)            │
  │  (speculare: y → -roll_tgt)                              │
  └──────────────────────────────────────────────────────────┘

  ATTITUDE CONTROL  (foto 2: Roll P / Pitch P → Rate PID)
  ┌──────────────────────────────────────────────────────────┐
  │  pitch_tgt ──►[Σ]──► P ──► pitch_rate_tgt               │
  │               ▲  (kp=4,    (sat ±2 rad/s)               │
  │             pitch  sat 2)                                │
  │                     pitch_rate_tgt ──►[Σ]──► PD ──► cmd │
  │                                      ▲  (kp=0.75,       │
  │                                   w_pitch  kd=0.05,      │
  │                                            sat ±2)       │
  │  (speculare per roll)                                    │
  └──────────────────────────────────────────────────────────┘

  TILT COMPENSATION + MIXER X-quad
  ┌──────────────────────────────────────────────────────────┐
  │  f = f_base / cos(tilt)  [clamp ≤ f_base × 1.3]        │
  │  f1 = f + roll_cmd - pitch_cmd   (destra-fronte)        │
  │  f2 = f - roll_cmd - pitch_cmd   (sinistra-fronte)      │
  │  f3 = f - roll_cmd + pitch_cmd   (sinistra-retro)       │
  │  f4 = f + roll_cmd + pitch_cmd   (destra-retro)         │
  │  Clamp finale: fi = max(0, fi)   ← motori non frenano   │
  └──────────────────────────────────────────────────────────┘
"""

import math
from controllers import P_Controller, PI_Controller, PID_Controller

# Stimato dai log: hover stabile → f_per_motor ≈ 3.59 N
# Aumenta se il drone scende lentamente in hover, riduci se sale.
HOVER_FF = 3.59


class MultirotorController:

    def __init__(self, hover_ff: float = HOVER_FF):
        self.hover_ff = hover_ff

        # QUOTA
        self.z_control  = P_Controller(kp=2.0, sat=2.0)
        self.vz_control = PI_Controller(kp=5.0, ki=2.0, sat=5.0)

        # POSIZIONE XY — solo P, nessun integrale (evita windup)
        self.x_control  = P_Controller(kp=0.5, sat=3.0) # prima: kp=0.5, sat=1.5
        self.y_control  = P_Controller(kp=0.5, sat=1.5)
        self.vx_control = P_Controller(kp=0.4, sat=math.radians(15))#prima 8
        self.vy_control = P_Controller(kp=0.4, sat=math.radians(8))

        # ATTITUDE — P per angolo, PD per rate (NO integrali nel loop veloce)
        self.roll_control  = P_Controller(kp=4.0, sat=2.0)
        self.pitch_control = P_Controller(kp=4.0, sat=2.0)
        self.w_roll_control  = PID_Controller(kp=0.75, ki=0.0, kd=0.05, sat=2.0)
        self.w_pitch_control = PID_Controller(kp=0.75, ki=0.0, kd=0.05, sat=2.0)

        

        # Setpoint
        self.z_target = 1.0
        self.x_target = 0.0
        self.y_target = 0.0

        # Valori intermedi per debug
        self.vz_target    = 0.0
        self.vx_target    = 0.0
        self.vy_target    = 0.0
        self.roll_target  = 0.0
        self.pitch_target = 0.0

    def evaluate(self,
                 delta_t: float,
                 z: float,  vz: float,
                 x: float,  vx: float,
                 y: float,  vy: float,
                 roll: float,  roll_rate: float,
                 pitch: float, pitch_rate: float,
                 altitude_only: bool = False) -> tuple:
        """
        Calcola (f1, f2, f3, f4) in Newton.
        altitude_only=True: usare SOLO durante TAKEOFF (non HOVERING).
        """

        # QUOTA (sempre attiva)
        self.vz_target = self.z_control.evaluate(delta_t, self.z_target - z)
        f_corr = self.vz_control.evaluate(delta_t, self.vz_target - vz)
        f_base = self.hover_ff + f_corr

        if altitude_only:
            # Solo durante TAKEOFF: forza uniforme, reset solo attitude
            self.w_roll_control.reset()
            self.w_pitch_control.reset()
            return f_base, f_base, f_base, f_base

        # POSIZIONE XY
        # Asse X (Destra/Sinistra) -> Controllato dal Roll
        self.vx_target   = self.x_control.evaluate(delta_t, self.x_target - x)
        self.roll_target = -self.vx_control.evaluate(delta_t, self.vx_target - vx) # <-- MENO

        # Asse Y del controller (Godot Z, Avanti/Indietro) -> Controllato dal Pitch
        self.vy_target    = self.y_control.evaluate(delta_t, self.y_target - y)
        self.pitch_target = self.vy_control.evaluate(delta_t, self.vy_target - vy) # <-- MENO TOLTO

        # ATTITUDE RATE
        pitch_rate_tgt = self.pitch_control.evaluate(delta_t, self.pitch_target - pitch)
        pitch_cmd      = self.w_pitch_control.evaluate(delta_t, pitch_rate_tgt - pitch_rate)

        roll_rate_tgt  = self.roll_control.evaluate(delta_t, self.roll_target - roll)
        roll_cmd       = self.w_roll_control.evaluate(delta_t, roll_rate_tgt - roll_rate)

        # TILT COMPENSATION
        tilt  = math.sqrt(roll * roll + pitch * pitch)
        cos_t = math.cos(tilt)
        f = min(f_base / cos_t, f_base * 1.3) if cos_t > 0.5 else f_base

     
        # MIXER (Compatibile con la fisica di Godot)
        f1 = f + roll_cmd - pitch_cmd
        f2 = f - roll_cmd - pitch_cmd
        f3 = f - roll_cmd + pitch_cmd
        f4 = f + roll_cmd + pitch_cmd

        # Clamp: i motori reali non producono spinta negativa
        f1 = max(0.0, f1)
        f2 = max(0.0, f2)
        f3 = max(0.0, f3)
        f4 = max(0.0, f4)

        return f1, f2, f3, f4

    def set_target(self, x=None, y=None, z=None):
        if x is not None: self.x_target = x
        if y is not None: self.y_target = y
        if z is not None: self.z_target = z