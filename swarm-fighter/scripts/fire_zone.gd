## fire_zone.gd — Singola zona di incendio nella scena.
##
## Struttura del nodo (da impostare nella scena fire_zone.tscn):
##   FireZone  (Node3D, questo script)
##   ├── Particles  (GPUParticles3D — fiamme visive)
##   ├── DetectionArea  (Area3D — rileva i droni vicini)
##   │   └── CollisionShape3D  (SphereShape3D, raggio = detection_radius)
##   └── Label3D  (opzionale — mostra fire_id sopra le fiamme)
##
## Quando un drone entra in DetectionArea, pubblica su DDS l'evento
## incendio. Il FireManager decide quando spawnare/rimuovere questa zona.

extends Node3D

# ---------------------------------------------------------------------------
# Parametri (impostabili dall'Inspector o da FireManager via codice)
# ---------------------------------------------------------------------------
@export var fire_id          : int   = 0
@export var detection_radius : float = 5.0    # [m]
@export var auto_extinguish  : bool  = false  # se true, Python non serve

# ---------------------------------------------------------------------------
# Riferimenti ai nodi figli (assegnati in _ready)
# ---------------------------------------------------------------------------
@onready var _particles : GPUParticles3D = $Particles
@onready var _area      : Area3D         = $DetectionArea
@onready var _label     : Label3D        = $Label3D   # può essere null

var _active      : bool = true
var _drones_near : int  = 0   # quanti droni sono attualmente vicini

# ---------------------------------------------------------------------------
# Segnali
# ---------------------------------------------------------------------------
signal extinguished(id: int)

# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

func _ready() -> void:
	_area.body_entered.connect(_on_body_entered)
	_area.body_exited.connect(_on_body_exited)

	# Aggiorna label se presente
	if _label:
		_label.text = "FIRE #%d" % fire_id

	# Ascolta il segnale di risoluzione da Python
	DDS.subscribe("world/fire_resolved")

	if _particles:
		_particles.emitting = true


func _process(_delta: float) -> void:
	if not _active:
		return

	# Controlla se Python ha segnalato che questo fuoco è spento
	var resolved_id := int(DDS.read("world/fire_resolved"))
	if resolved_id == fire_id:
		_extinguish()


# ---------------------------------------------------------------------------
# Rilevamento droni
# ---------------------------------------------------------------------------

func _on_body_entered(body: Node3D) -> void:
	## Chiamato quando un RigidBody3D entra nell'area di rilevamento.
	## Verifica che sia effettivamente un drone (ha la variabile drone_id).
	if not _active:
		return
	if not body.has_method("reset"):   # check leggero: tutti i droni hanno reset()
		return

	_drones_near += 1

	# Pubblica evento incendio su DDS (tutti gli agenti Python lo riceveranno)
	# Pubblichiamo ogni volta che un drone entra, così se un agente era
	# offline alla prima pubblicazione lo riceve comunque.
	_publish_fire_event()


func _on_body_exited(body: Node3D) -> void:
	if not body.has_method("reset"):
		return
	_drones_near = max(0, _drones_near - 1)


func _publish_fire_event() -> void:
	DDS.publish("world/fire_new",   DDS.DDS_TYPE_FLOAT, float(fire_id))
	DDS.publish("world/fire_x",     DDS.DDS_TYPE_FLOAT, global_position.x)
	DDS.publish("world/fire_y",     DDS.DDS_TYPE_FLOAT, global_position.y)
	DDS.publish("world/fire_z",     DDS.DDS_TYPE_FLOAT, global_position.z)


# ---------------------------------------------------------------------------
# Spegnimento
# ---------------------------------------------------------------------------

func _extinguish() -> void:
	_active = false
	if _particles:
		_particles.emitting = false

	# Resetta il topic in modo che gli agenti non rispondano più a questo fuoco
	DDS.clear("world/fire_new")
	DDS.clear("world/fire_resolved")

	emit_signal("extinguished", fire_id)

	# Rimuovi il nodo dopo un piccolo delay (le particelle finiscono di emettere)
	await get_tree().create_timer(2.0).timeout
	queue_free()


# ---------------------------------------------------------------------------
# API pubblica (usata da FireManager)
# ---------------------------------------------------------------------------

func activate(id: int, pos: Vector3) -> void:
	fire_id          = id
	global_position  = pos
	_active          = true
	if _particles:
		_particles.emitting = true
	if _label:
		_label.text = "FIRE #%d" % fire_id
	# Pubblica immediatamente l'evento così Python sa che c'è un nuovo fuoco
	_publish_fire_event()
