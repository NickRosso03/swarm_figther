## drone.gd — Corpo fisico del drone in Godot 4.5
##
## Pubblica verso Python ogni physics frame:
##   drone_{id}/X,Y,Z   posizione world [m]
##   drone_{id}/VX,VY,VZ velocità lineare [m/s]
##   drone_{id}/TX,TY,TZ angoli Euler roll/pitch/yaw [rad]
##   drone_{id}/WX,WY,WZ velocità angolare [rad/s]
##   drone_{id}/connected 1.0 ogni frame (Python aspetta questo per partire)
##   drone_{id}/tick     impulso di sync — SEMPRE l'ultimo publish
##
## Legge da Python ogni physics frame:
##   drone_{id}/f1..f4  forze propulsori [N]
##
## Disposizione motori (vista dall'alto):
##   2 --- 1
##   |     |
##   3 --- 4
##   p1=( L,0, L)  p2=(-L,0, L)  p3=(-L,0,-L)  p4=( L,0,-L)

extends RigidBody3D

@export var drone_id   : int   = 0
@export var arm_length : float = 0.195

@onready var mini_hud : Label3D = $MiniHUD
@onready var water_cannon : Node3D = $WaterCannon
@onready var foam_particles : GPUParticles3D = $WaterCannon/FoamParticles

const STATUS_NAMES := {
	0.0: "IDLE/TAKEOFF",
	1.0: "EXPLORING",
	2.0: "MOVING",
	3.0: "RETURNING",
	4.0: "SUPPRESSING" # Nuovo stato
}

var _prefix  : String
var _p1      : Vector3
var _p2      : Vector3
var _p3      : Vector3
var _p4      : Vector3

var _f1      : float = 0.0
var _f2      : float = 0.0
var _f3      : float = 0.0
var _f4      : float = 0.0

var _initial_pos : Vector3
var _initial_rot : Vector3
var _dbg_frame   : int = 0


func _ready() -> void:
	_prefix = "drone_%d" % drone_id
	_p1 = Vector3( arm_length, 0.0,  arm_length)
	_p2 = Vector3(-arm_length, 0.0,  arm_length)
	_p3 = Vector3(-arm_length, 0.0, -arm_length)
	_p4 = Vector3( arm_length, 0.0, -arm_length)

	_initial_pos = global_position
	_initial_rot = global_rotation

	DDS.subscribe("%s/f1" % _prefix)
	DDS.subscribe("%s/f2" % _prefix)
	DDS.subscribe("%s/f3" % _prefix)
	DDS.subscribe("%s/f4" % _prefix)
	
	DDS.subscribe("%s/status" % _prefix)
	DDS.subscribe("%s/fire_x" % _prefix)
	DDS.subscribe("%s/fire_y" % _prefix)
	DDS.subscribe("%s/fire_z" % _prefix)
	
	

func _physics_process(_delta: float) -> void:
	_f1 = DDS.read("%s/f1" % _prefix)
	_f2 = DDS.read("%s/f2" % _prefix)
	_f3 = DDS.read("%s/f3" % _prefix)
	_f4 = DDS.read("%s/f4" % _prefix)

	# --- DEBUG: stampa ogni 120 frame (~2s) solo il drone 0 ---
	_dbg_frame += 1
	if drone_id == 0 and _dbg_frame % 120 == 0:
		print("[drone_0] pos=(%.2f, %.2f, %.2f)  f1=%.3f f2=%.3f f3=%.3f f4=%.3f" % [
			global_position.x, global_position.y, global_position.z,
			_f1, _f2, _f3, _f4
		])

	_apply_motor_force(_f1, _p1)
	_apply_motor_force(_f2, _p2)
	_apply_motor_force(_f3, _p3)
	_apply_motor_force(_f4, _p4)
	
	# Coppia aerodinamica moderata
	#var drag_coeff := 0.1  
	#var yaw_torque := (_f1 - _f2 + _f3 - _f4) * drag_coeff
	#apply_torque(transform.basis * Vector3(0.0, yaw_torque, 0.0))
	
	

	_publish_state()


func _publish_state() -> void:
	var wp : Vector3 = global_position
	var wv : Vector3 = linear_velocity
	var wr : Vector3 = global_rotation
	var wa : Vector3 = angular_velocity

	DDS.publish("%s/X"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.x)
	DDS.publish("%s/Y"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.y)
	DDS.publish("%s/Z"  % _prefix, DDS.DDS_TYPE_FLOAT, wp.z)
	DDS.publish("%s/VX" % _prefix, DDS.DDS_TYPE_FLOAT, wv.x)
	DDS.publish("%s/VY" % _prefix, DDS.DDS_TYPE_FLOAT, wv.y)
	DDS.publish("%s/VZ" % _prefix, DDS.DDS_TYPE_FLOAT, wv.z)
	DDS.publish("%s/TX" % _prefix, DDS.DDS_TYPE_FLOAT, wr.x)
	DDS.publish("%s/TY" % _prefix, DDS.DDS_TYPE_FLOAT, wr.y)
	DDS.publish("%s/TZ" % _prefix, DDS.DDS_TYPE_FLOAT, wr.z)
	DDS.publish("%s/WX" % _prefix, DDS.DDS_TYPE_FLOAT, wa.x)
	DDS.publish("%s/WY" % _prefix, DDS.DDS_TYPE_FLOAT, wa.y)
	DDS.publish("%s/WZ" % _prefix, DDS.DDS_TYPE_FLOAT, wa.z)

	# connected ogni frame: Python potrebbe connettersi in qualsiasi momento
	DDS.publish("%s/connected" % _prefix, DDS.DDS_TYPE_FLOAT, 1.0)

	# tick SEMPRE per ultimo: è il segnale che sincronizza il loop Python
	DDS.publish("%s/tick" % _prefix, DDS.DDS_TYPE_FLOAT, 1.0)


func _apply_motor_force(force_n: float, local_pos: Vector3) -> void:
	var world_pos   : Vector3 = transform.basis * local_pos
	var world_force : Vector3 = transform.basis * Vector3(0.0, force_n, 0.0)
	apply_force(world_force, world_pos)


func reset() -> void:
	global_position  = _initial_pos
	global_rotation  = _initial_rot
	linear_velocity  = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_f1 = 0.0; _f2 = 0.0; _f3 = 0.0; _f4 = 0.0
	DDS.clear("%s/f1" % _prefix)
	DDS.clear("%s/f2" % _prefix)
	DDS.clear("%s/f3" % _prefix)
	DDS.clear("%s/f4" % _prefix)
	
	
func _process(_delta: float) -> void:
	# 1. Aggiorna il Mini HUD
	var status : float = DDS.read("%s/status" % _prefix)
	mini_hud.text = STATUS_NAMES.get(status, "UNK") + _prefix
	
	# 2. Gestisci le particelle d'acqua
	if status == 4.0: # SUPPRESSING
		if not foam_particles.emitting:
			foam_particles.emitting = true
			
		# Leggi le coordinate dell'incendio assegnato al drone
		var fx = DDS.read("%s/fire_x" % _prefix)
		var fy = DDS.read("%s/fire_y" % _prefix)
		var fz = DDS.read("%s/fire_z" % _prefix)
		var fire_pos = Vector3(fx, fy, fz)
		
		# Fai ruotare il "cannone" per guardare verso l'incendio.
		# L'asse Z negativo (-Z) del WaterCannon punterà dritto verso il fuoco.
		water_cannon.look_at(fire_pos, Vector3.UP)
	else:
		if foam_particles.emitting:
			foam_particles.emitting = false
