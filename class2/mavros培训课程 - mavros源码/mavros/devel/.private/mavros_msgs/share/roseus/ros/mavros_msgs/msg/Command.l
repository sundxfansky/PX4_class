;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::Command)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'Command (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::COMMAND")
  (make-package "MAVROS_MSGS::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mavros_msgs::Command
  :super ros::object
  :slots (_header _comid _command _sub_mode _pos_sp _vel_sp _yaw_sp _yaw_rate_sp ))

(defmethod mavros_msgs::Command
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:comid __comid) 0)
    ((:command __command) 0)
    ((:sub_mode __sub_mode) 0)
    ((:pos_sp __pos_sp) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vel_sp __vel_sp) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:yaw_sp __yaw_sp) 0.0)
    ((:yaw_rate_sp __yaw_rate_sp) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _comid (round __comid))
   (setq _command (round __command))
   (setq _sub_mode (round __sub_mode))
   (setq _pos_sp __pos_sp)
   (setq _vel_sp __vel_sp)
   (setq _yaw_sp (float __yaw_sp))
   (setq _yaw_rate_sp (float __yaw_rate_sp))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:comid
   (&optional __comid)
   (if __comid (setq _comid __comid)) _comid)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:sub_mode
   (&optional __sub_mode)
   (if __sub_mode (setq _sub_mode __sub_mode)) _sub_mode)
  (:pos_sp
   (&optional __pos_sp)
   (if __pos_sp (setq _pos_sp __pos_sp)) _pos_sp)
  (:vel_sp
   (&optional __vel_sp)
   (if __vel_sp (setq _vel_sp __vel_sp)) _vel_sp)
  (:yaw_sp
   (&optional __yaw_sp)
   (if __yaw_sp (setq _yaw_sp __yaw_sp)) _yaw_sp)
  (:yaw_rate_sp
   (&optional __yaw_rate_sp)
   (if __yaw_rate_sp (setq _yaw_rate_sp __yaw_rate_sp)) _yaw_rate_sp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _comid
    4
    ;; uint8 _command
    1
    ;; uint8 _sub_mode
    1
    ;; float32[3] _pos_sp
    (* 4    3)
    ;; float32[3] _vel_sp
    (* 4    3)
    ;; float32 _yaw_sp
    4
    ;; float32 _yaw_rate_sp
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _comid
       (write-long _comid s)
     ;; uint8 _command
       (write-byte _command s)
     ;; uint8 _sub_mode
       (write-byte _sub_mode s)
     ;; float32[3] _pos_sp
     (dotimes (i 3)
       (sys::poke (elt _pos_sp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _vel_sp
     (dotimes (i 3)
       (sys::poke (elt _vel_sp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yaw_sp
       (sys::poke _yaw_sp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate_sp
       (sys::poke _yaw_rate_sp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _comid
     (setq _comid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sub_mode
     (setq _sub_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32[3] _pos_sp
   (dotimes (i (length _pos_sp))
     (setf (elt _pos_sp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _vel_sp
   (dotimes (i (length _vel_sp))
     (setf (elt _vel_sp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _yaw_sp
     (setq _yaw_sp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate_sp
     (setq _yaw_rate_sp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mavros_msgs::Command :md5sum-) "fc0a85df8be3034122897f9d6b86110f")
(setf (get mavros_msgs::Command :datatype-) "mavros_msgs/Command")
(setf (get mavros_msgs::Command :definition-)
      "
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



================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :mavros_msgs/Command "fc0a85df8be3034122897f9d6b86110f")


