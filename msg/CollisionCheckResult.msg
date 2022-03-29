Header header

# motion data of the object that causes collision check
PerceivedObjectMotion perceived_object

# time-to-collision for the object
float32 ttc

# type of the results (one of the following constants)
uint8 result_type

uint8 RESULT_IGNORE = 0
uint8 RESULT_CLOSE_MONITORING = 1
uint8 RESULT_WARNING = 2
uint8 RESULT_ALERT = 3
