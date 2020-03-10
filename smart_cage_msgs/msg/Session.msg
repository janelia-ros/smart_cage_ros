string mouse_name

# range(24) 9 = 9:00AM, 15 = 3:00PM
uint8 start_time_hour 9

# range(60) 0 = 9:00AM, 30 = 9:30AM
uint8 start_time_minute 0

# duration (hours) (duration + start_time_hour) must be <= 23
uint8 duration 12

float32 threshold 0.25

bool latch_mode false

bool require_both_head_bar_sensors true
