# header, contains time stamp
Header header

# ID of the object
uint32 id

# position of vehicle in (x,y)-plane, z = 0 [m]
geometry_msgs/Point position

# heading of vehicle relative to y-axis [degree]
float64 heading

# speed of vehicle [m/s]
float64 speed

# acceleration of vehicle [m/s^2]
float64 acceleration