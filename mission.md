start randomly from any poin in the arena
arena size is 60 x 30 meters
have to detect person using yolo.

once a person is detected, approach it: fly slowly toward the person, keeping
the bottom-centre of its bounding box at the centre of the frame (yaw to track,
creep forward), until the front LiDAR distance is 3 m — then stop.

skip rtl for now
