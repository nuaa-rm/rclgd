extends Ros2Publisher

func _physics_process(delta):
	if Engine.get_physics_frames() % 60 == 0:
		publish({ "header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "" }, "height": 2, "width": 2, "distortion_model": "plumb_bob", "d": [0.0, 0.0, 0.0, 0.0, 0.0], "k": [1.30322537284121, 0.0, 1.0, 0.0, 1.95483805926181, 1.0, 0.0, 0.0, 1.0], "r": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "binning_x": 0, "binning_y": 0, "roi": { "x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": false } })
	
