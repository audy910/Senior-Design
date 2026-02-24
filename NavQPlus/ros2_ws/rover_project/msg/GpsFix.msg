# GPS fix message with comprehensive GPS state
float64 latitude        # Latitude in degrees (WGS84)
float64 longitude       # Longitude in degrees (WGS84)
float32 speed           # Speed over ground in m/s
float32 course          # Course over ground in degrees (0-360)
uint8 fix_type          # Fix type: 0=none, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+RTK, 5=RTK float
uint8 num_sats          # Number of satellites used
float32 hdop            # Horizontal dilution of precision
float32 h_acc           # Horizontal accuracy estimate in meters
float32 v_acc           # Vertical accuracy estimate in meters
