## world.gd — Script della scena principale.
##
## Responsabilità:
##   - Istanzia i 5 droni a posizioni iniziali sfalsate
##   - Espone il bottone di reset
##   - Mostra un HUD minimale con lo stato di ogni drone via Label
##
## Struttura della scena world.tscn:
##   World  (Node3D, questo script)
##   ├── DDS           (dds.gd — Autoload, non serve aggiungerlo qui
##   │                  se è registrato come Autoload in Project Settings)
##   ├── Environment   (WorldEnvironment + DirectionalLight3D)
##   ├── Ground        (StaticBody3D con CollisionShape3D piatta)
##   ├── FireManager   (Node3D, fire_manager.gd)
##   ├── Drones        (Node3D — contenitore, figli aggiunti dinamicamente)
##   └── UI            (CanvasLayer)
##       ├── ResetButton (Button)
##       └── StatusPanel (VBoxContainer con 5 Label)

extends Node3D

# ---------------------------------------------------------------------------
# Parametri
# ---------------------------------------------------------------------------
@export var drone_scene    : PackedScene          # assegna drone.tscn
@export var n_drones       : int   = 5
@export var area_size      : float = 80.0         # deve coincidere con Python
@export var start_altitude : float = 0.3          # piccolo offset da terra

# ---------------------------------------------------------------------------
# Riferimenti
# ---------------------------------------------------------------------------
@onready var _drones_root : Node3D    = $Drones
@onready var _status_panel: VBoxContainer = $UI/StatusPanel
@onready var _reset_btn   : Button    = $UI/ResetButton

var _drones    : Array = []
var _labels    : Array = []

# Mapping codice numerico → stringa leggibile (deve coincidere con drone_agent.py)
const STATUS_NAMES := {
	0.0: "IDLE/TAKEOFF",
	1.0: "EXPLORING",
	2.0: "MOVING/SUPP.",
	3.0: "RETURNING",
}

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	if drone_scene == null:
		push_error("world.gd: drone_scene non assegnata!")
		return

	_spawn_drones()
	_build_hud()

	_reset_btn.pressed.connect(_on_reset_pressed)

	# Registra le variabili di stato dei droni per l'HUD
	for i in n_drones:
		DDS.subscribe("drone_%d/status" % i)


func _process(_delta: float) -> void:
	_update_hud()


# ---------------------------------------------------------------------------
# Spawn droni
# ---------------------------------------------------------------------------

func _spawn_drones() -> void:
	var spacing := area_size / n_drones
	for i in n_drones:
		var drone : RigidBody3D = drone_scene.instantiate()
		drone.drone_id = i
		drone.name     = "Drone%d" % i
		_drones_root.add_child(drone)
		# global_position va impostata DOPO add_child(),
		# solo quando il nodo è dentro l'albero della scena
		drone.global_position = Vector3(
			i * spacing + spacing * 0.5,
			start_altitude,
			2.0    # leggermente dentro l'area
		)
		_drones.append(drone)
		print("World: Drone %d spawned in (%.1f, %.1f, %.1f)" \
			  % [i, drone.global_position.x,
					drone.global_position.y,
					drone.global_position.z])


# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------

func _build_hud() -> void:
	for i in n_drones:
		var lbl := Label.new()
		lbl.text             = "D%d: --" % i
		lbl.add_theme_font_size_override("font_size", 14)
		_status_panel.add_child(lbl)
		_labels.append(lbl)


func _update_hud() -> void:
	for i in n_drones:
		var code   : float  = DDS.read("drone_%d/status" % i)
		var _name   : String = STATUS_NAMES.get(code, "?")
		var drone  : Node3D = _drones[i]
		var pos    : Vector3 = drone.global_position
		_labels[i].text = "D%d [%s] (%.0f, %.0f, %.0f)" \
			% [i, _name, pos.x, pos.y, pos.z]


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

func _on_reset_pressed() -> void:
	for drone in _drones:
		drone.reset()
	print("World: tutti i droni resettati.")
