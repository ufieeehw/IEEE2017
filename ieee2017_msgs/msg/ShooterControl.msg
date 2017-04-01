uint8 pan_center=114	# The center positions for the pan and tilt servos;
uint8 tilt_center=95	# positions the shooter horizontal and facing forward

uint8 dart_1=10		# The servo angles for each dart in the clip
uint8 dart_2=85
uint8 dart_3=163

uint8 pan_angle		# Pan and tilt angles based in a coordinate system
uint8 tilt_angle	# where the centered shooter is at (0, 0)

uint8 dart		# Number of the dart to select in range [0, 2]
bool flywheels		# Activate the flywheels while true
bool fire		# Activate the shooter on switch from false to true
