## fire_manager.gd — Gestore degli incendi dinamici.
##
## Spawna nuovi incendi in posizioni casuali all'interno dell'area di gioco,
## con un intervallo random tra min_interval e max_interval secondi.
## Tiene traccia degli incendi attivi e non ne spawna troppi contemporaneamente.
##
## Struttura consigliata nella scena:
##   FireManager  (Node3D, questo script)
##   └── (i FireZone vengono aggiunti come figli dinamicamente)

extends Node3D

# ---------------------------------------------------------------------------
# Parametri (modificabili dall'Inspector)
# ---------------------------------------------------------------------------
@export var fire_zone_scene  : PackedScene          # assegna fire_zone.tscn
@export var area_min         : Vector2 = Vector2(-75.0,  75.0)   # angolo SW area
@export var area_max         : Vector2 = Vector2(75.0, -75.0)  # angolo NE area
@export var min_interval     : float   = 10.0  # [s] intervallo minimo tra spawn
@export var max_interval     : float   = 25.0  # [s] intervallo massimo
@export var max_active_fires : int     = 3     # incendi contemporanei massimi
@export var fire_altitude    : float   = 0.0   # Y dove spawna il fuoco

# ---------------------------------------------------------------------------
# Stato interno
# ---------------------------------------------------------------------------
var _next_fire_id  : int   = 1
var _active_fires  : Dictionary = {}   # fire_id → FireZone node
var _spawn_timer   : float = 0.0
var _next_spawn_at : float = 0.0

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	randomize()
	_schedule_next_spawn()
	print("FireManager: pronto. Primo incendio tra %.1f s" % _next_spawn_at)


func _process(delta: float) -> void:
	_spawn_timer += delta

	if _spawn_timer >= _next_spawn_at:
		_spawn_timer = 0.0
		_try_spawn_fire()
		_schedule_next_spawn()


# ---------------------------------------------------------------------------
# Spawn
# ---------------------------------------------------------------------------

func _try_spawn_fire() -> void:
	if _active_fires.size() >= max_active_fires:
		print("FireManager: troppi incendi attivi (%d), skip spawn." \
			  % _active_fires.size())
		return

	if fire_zone_scene == null:
		push_error("FireManager: fire_zone_scene non assegnata!")
		return

	var pos := _random_position()
	var id  := _next_fire_id
	_next_fire_id += 1

	var zone : Node3D = fire_zone_scene.instantiate()
	add_child(zone)
	zone.extinguished.connect(_on_fire_extinguished)
	zone.activate(id, pos)

	_active_fires[id] = zone
	print("FireManager: nuovo incendio #%d in (%.1f, %.1f, %.1f)" \
		  % [id, pos.x, pos.y, pos.z])


func _random_position() -> Vector3:
	var x := randf_range(area_min.x, area_max.x)
	var z := randf_range(area_min.y, area_max.y)
	return Vector3(x, fire_altitude, z)


func _schedule_next_spawn() -> void:
	_next_spawn_at = randf_range(min_interval, max_interval)


# ---------------------------------------------------------------------------
# Callback spegnimento
# ---------------------------------------------------------------------------

func _on_fire_extinguished(id: int) -> void:
	_active_fires.erase(id)
	print("FireManager: incendio #%d rimosso. Attivi: %d" \
		  % [id, _active_fires.size()])
