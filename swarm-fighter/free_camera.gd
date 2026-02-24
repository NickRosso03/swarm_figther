## free_camera.gd — Camera free-look per debug della scena.
##
## Controlli:
##   Tasto destro hold → abilita movimento
##   WASD              → muovi orizzontalmente
##   Q / E             → scendi / sali
##   Scroll wheel      → aumenta/diminuisce velocità
##   Shift             → velocità x4
##   F                 → torna alla posizione di overview

extends Camera3D

@export var move_speed   : float = 20.0
@export var look_sens    : float = 0.003
@export var scroll_step  : float = 5.0

var _active : bool = false   # true solo mentre tasto destro è premuto


func _ready() -> void:
	make_current()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		match event.button_index:
			MOUSE_BUTTON_RIGHT:
				_active = event.pressed
				Input.set_mouse_mode(
					Input.MOUSE_MODE_CAPTURED if _active \
					else Input.MOUSE_MODE_VISIBLE
				)
			MOUSE_BUTTON_WHEEL_UP:
				move_speed = minf(move_speed + scroll_step, 200.0)
			MOUSE_BUTTON_WHEEL_DOWN:
				move_speed = maxf(move_speed - scroll_step, 1.0)

	if event is InputEventMouseMotion and _active:
		rotate_y(-event.relative.x * look_sens)
		rotate_object_local(Vector3.RIGHT, -event.relative.y * look_sens)
		var euler_rot := rotation
		euler_rot.x   = clampf(euler_rot.x, deg_to_rad(-89.0), deg_to_rad(89.0))
		rotation       = euler_rot

	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_F:
			global_position = Vector3(40.0, 40.0, 80.0)
			rotation        = Vector3(deg_to_rad(-25.0), 0.0, 0.0)
			print("Camera: reset overview")


func _process(delta: float) -> void:
	if not _active:
		return

	var spd : float = move_speed * (4.0 if Input.is_key_pressed(KEY_SHIFT) else 1.0)
	var dir : Vector3 = Vector3.ZERO

	if Input.is_key_pressed(KEY_W): dir -= transform.basis.z
	if Input.is_key_pressed(KEY_S): dir += transform.basis.z
	if Input.is_key_pressed(KEY_A): dir -= transform.basis.x
	if Input.is_key_pressed(KEY_D): dir += transform.basis.x
	if Input.is_key_pressed(KEY_E): dir += Vector3.UP
	if Input.is_key_pressed(KEY_Q): dir -= Vector3.UP

	if dir != Vector3.ZERO:
		global_position += dir.normalized() * spd * delta
