float32 roll 		# Camera roll angle (deg, -180..+180), float
float32 pitch 	# Camera pitch angle (deg, -180..+180), float
float32 FOV 		# Camera horizontal FOV in deg, float
uint8 tracker_status # Status of the object tracker:
uint8 TRACKER_STATUS_IDLE = 0			# Tracker is Idle, ready for all tracking	commands
uint8 TRACKER_STATUS_CROSSHAIR = 1	# Crosshair is displayed, ready for track on command
uint8 TRACKER_STATUS_ACTIVE = 2		# Tracker is actively tracking an object
uint8 TRACKER_STATUS_RETRACK = 3		# Re-Track crosshair is displayed, movable by the user, ready for track on command
uint8 TRACKER_STATUS_ACTIVE4 = 4		# Tracker is actively tracking an object
uint8 TRACKER_STATUS_ACTIVE5 = 5		# Tracker is actively tracking an object */
uint8 recording_status	 # Status of the video recording:
uint8 RECORDING_STATUS_IDLE = 0  # Idle, not recording Detected
uint8 RECORDING_STATUS_ACTIVE = 1 #Active, video is being recorded Detected
uint8 RECORDING_STATUS_DISBALED = 2 #Disabled Not Detected 
uint8 sensor_active		# 0= Day Sensor, 1=IR Sensor
uint8 sensor_polarity # 0=white hot, 1= black hot
uint8 system_mode
uint8 laser_status
uint16 tracker_roix # X offset of the center of the tracker ROI, measured from the top left corner of the frame.
uint16 tracker_rioy # Y offset of the center of the tracker ROI, measured from the top left corner of the frame.