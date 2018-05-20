
std_msgs/Header header

#enum Command
#{
#    Standby = 0,
#    Takeoff,
#    Hold,
#    Land,
#    Move,
#    Disarm,
#    Moving_Body
#};
# sub_mode 2-bit value: 
# 0 for position, 1 for vel, 1st for xy, 2nd for z.
#                   xy position     xy velocity
# z position       	0b00(0)       0b10(2)
# z velocity		0b01(1)       0b11(3)
#

uint32 comid
uint8 command
uint8 sub_mode
float32[3] pos_sp
float32[3] vel_sp
float32 yaw_sp
float32 yaw_rate_sp


