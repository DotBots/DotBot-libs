# Wheel speed loop

Drives a 500 mm square forward, then the same square backward, on the per-wheel
speed loop. Each side and each corner is an odometric goal: the wheels run until
the encoders say the distance is done, then brake to a stand. A stalled wheel
ends the sequence there.
