;; Auto-generated. Do not edit!


(when (boundp 'space_cobot_mpc_controller::pwm_values)
  (if (not (find-package "SPACE_COBOT_MPC_CONTROLLER"))
    (make-package "SPACE_COBOT_MPC_CONTROLLER"))
  (shadow 'pwm_values (find-package "SPACE_COBOT_MPC_CONTROLLER")))
(unless (find-package "SPACE_COBOT_MPC_CONTROLLER::PWM_VALUES")
  (make-package "SPACE_COBOT_MPC_CONTROLLER::PWM_VALUES"))

(in-package "ROS")
;;//! \htmlinclude pwm_values.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass space_cobot_mpc_controller::pwm_values
  :super ros::object
  :slots (_header _Force _Moment _Actuation _PWM _rpm _Att_P_gains _Att_I_gains _Att_D_gains _Current_rpy _Desired_rpy _Desired_position _Desired_velocity _Desired_omega _Error_rpy _err_int _Time_Step _Quat_error ))

(defmethod space_cobot_mpc_controller::pwm_values
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:Force __Force) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Moment __Moment) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Actuation __Actuation) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:PWM __PWM) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:rpm __rpm) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:Att_P_gains __Att_P_gains) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Att_I_gains __Att_I_gains) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Att_D_gains __Att_D_gains) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Current_rpy __Current_rpy) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Desired_rpy __Desired_rpy) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Desired_position __Desired_position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Desired_velocity __Desired_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Desired_omega __Desired_omega) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Error_rpy __Error_rpy) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:err_int __err_int) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:Time_Step __Time_Step) 0.0)
    ((:Quat_error __Quat_error) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _Force __Force)
   (setq _Moment __Moment)
   (setq _Actuation __Actuation)
   (setq _PWM __PWM)
   (setq _rpm __rpm)
   (setq _Att_P_gains __Att_P_gains)
   (setq _Att_I_gains __Att_I_gains)
   (setq _Att_D_gains __Att_D_gains)
   (setq _Current_rpy __Current_rpy)
   (setq _Desired_rpy __Desired_rpy)
   (setq _Desired_position __Desired_position)
   (setq _Desired_velocity __Desired_velocity)
   (setq _Desired_omega __Desired_omega)
   (setq _Error_rpy __Error_rpy)
   (setq _err_int __err_int)
   (setq _Time_Step (float __Time_Step))
   (setq _Quat_error __Quat_error)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:Force
   (&optional __Force)
   (if __Force (setq _Force __Force)) _Force)
  (:Moment
   (&optional __Moment)
   (if __Moment (setq _Moment __Moment)) _Moment)
  (:Actuation
   (&optional __Actuation)
   (if __Actuation (setq _Actuation __Actuation)) _Actuation)
  (:PWM
   (&optional __PWM)
   (if __PWM (setq _PWM __PWM)) _PWM)
  (:rpm
   (&optional __rpm)
   (if __rpm (setq _rpm __rpm)) _rpm)
  (:Att_P_gains
   (&optional __Att_P_gains)
   (if __Att_P_gains (setq _Att_P_gains __Att_P_gains)) _Att_P_gains)
  (:Att_I_gains
   (&optional __Att_I_gains)
   (if __Att_I_gains (setq _Att_I_gains __Att_I_gains)) _Att_I_gains)
  (:Att_D_gains
   (&optional __Att_D_gains)
   (if __Att_D_gains (setq _Att_D_gains __Att_D_gains)) _Att_D_gains)
  (:Current_rpy
   (&optional __Current_rpy)
   (if __Current_rpy (setq _Current_rpy __Current_rpy)) _Current_rpy)
  (:Desired_rpy
   (&optional __Desired_rpy)
   (if __Desired_rpy (setq _Desired_rpy __Desired_rpy)) _Desired_rpy)
  (:Desired_position
   (&optional __Desired_position)
   (if __Desired_position (setq _Desired_position __Desired_position)) _Desired_position)
  (:Desired_velocity
   (&optional __Desired_velocity)
   (if __Desired_velocity (setq _Desired_velocity __Desired_velocity)) _Desired_velocity)
  (:Desired_omega
   (&optional __Desired_omega)
   (if __Desired_omega (setq _Desired_omega __Desired_omega)) _Desired_omega)
  (:Error_rpy
   (&optional __Error_rpy)
   (if __Error_rpy (setq _Error_rpy __Error_rpy)) _Error_rpy)
  (:err_int
   (&optional __err_int)
   (if __err_int (setq _err_int __err_int)) _err_int)
  (:Time_Step
   (&optional __Time_Step)
   (if __Time_Step (setq _Time_Step __Time_Step)) _Time_Step)
  (:Quat_error
   (&optional __Quat_error)
   (if __Quat_error (setq _Quat_error __Quat_error)) _Quat_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _Force
    (* 8    3)
    ;; float64[3] _Moment
    (* 8    3)
    ;; float64[6] _Actuation
    (* 8    6)
    ;; float64[6] _PWM
    (* 8    6)
    ;; float64[6] _rpm
    (* 8    6)
    ;; float64[3] _Att_P_gains
    (* 8    3)
    ;; float64[3] _Att_I_gains
    (* 8    3)
    ;; float64[3] _Att_D_gains
    (* 8    3)
    ;; float64[3] _Current_rpy
    (* 8    3)
    ;; float64[3] _Desired_rpy
    (* 8    3)
    ;; float64[3] _Desired_position
    (* 8    3)
    ;; float64[3] _Desired_velocity
    (* 8    3)
    ;; float64[3] _Desired_omega
    (* 8    3)
    ;; float64[3] _Error_rpy
    (* 8    3)
    ;; float64[3] _err_int
    (* 8    3)
    ;; float64 _Time_Step
    8
    ;; float64[3] _Quat_error
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _Force
     (dotimes (i 3)
       (sys::poke (elt _Force i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Moment
     (dotimes (i 3)
       (sys::poke (elt _Moment i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _Actuation
     (dotimes (i 6)
       (sys::poke (elt _Actuation i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _PWM
     (dotimes (i 6)
       (sys::poke (elt _PWM i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _rpm
     (dotimes (i 6)
       (sys::poke (elt _rpm i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Att_P_gains
     (dotimes (i 3)
       (sys::poke (elt _Att_P_gains i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Att_I_gains
     (dotimes (i 3)
       (sys::poke (elt _Att_I_gains i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Att_D_gains
     (dotimes (i 3)
       (sys::poke (elt _Att_D_gains i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Current_rpy
     (dotimes (i 3)
       (sys::poke (elt _Current_rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Desired_rpy
     (dotimes (i 3)
       (sys::poke (elt _Desired_rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Desired_position
     (dotimes (i 3)
       (sys::poke (elt _Desired_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Desired_velocity
     (dotimes (i 3)
       (sys::poke (elt _Desired_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Desired_omega
     (dotimes (i 3)
       (sys::poke (elt _Desired_omega i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _Error_rpy
     (dotimes (i 3)
       (sys::poke (elt _Error_rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _err_int
     (dotimes (i 3)
       (sys::poke (elt _err_int i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _Time_Step
       (sys::poke _Time_Step (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _Quat_error
     (dotimes (i 3)
       (sys::poke (elt _Quat_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _Force
   (dotimes (i (length _Force))
     (setf (elt _Force i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Moment
   (dotimes (i (length _Moment))
     (setf (elt _Moment i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _Actuation
   (dotimes (i (length _Actuation))
     (setf (elt _Actuation i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _PWM
   (dotimes (i (length _PWM))
     (setf (elt _PWM i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _rpm
   (dotimes (i (length _rpm))
     (setf (elt _rpm i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Att_P_gains
   (dotimes (i (length _Att_P_gains))
     (setf (elt _Att_P_gains i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Att_I_gains
   (dotimes (i (length _Att_I_gains))
     (setf (elt _Att_I_gains i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Att_D_gains
   (dotimes (i (length _Att_D_gains))
     (setf (elt _Att_D_gains i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Current_rpy
   (dotimes (i (length _Current_rpy))
     (setf (elt _Current_rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Desired_rpy
   (dotimes (i (length _Desired_rpy))
     (setf (elt _Desired_rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Desired_position
   (dotimes (i (length _Desired_position))
     (setf (elt _Desired_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Desired_velocity
   (dotimes (i (length _Desired_velocity))
     (setf (elt _Desired_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Desired_omega
   (dotimes (i (length _Desired_omega))
     (setf (elt _Desired_omega i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _Error_rpy
   (dotimes (i (length _Error_rpy))
     (setf (elt _Error_rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _err_int
   (dotimes (i (length _err_int))
     (setf (elt _err_int i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _Time_Step
     (setq _Time_Step (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _Quat_error
   (dotimes (i (length _Quat_error))
     (setf (elt _Quat_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get space_cobot_mpc_controller::pwm_values :md5sum-) "da432c94c450f85a4b2d4b18064bf11c")
(setf (get space_cobot_mpc_controller::pwm_values :datatype-) "space_cobot_mpc_controller/pwm_values")
(setf (get space_cobot_mpc_controller::pwm_values :definition-)
      "Header header
float64[3] Force
float64[3] Moment
float64[6] Actuation
float64[6] PWM
float64[6] rpm
float64[3] Att_P_gains
float64[3] Att_I_gains
float64[3] Att_D_gains
float64[3] Current_rpy
float64[3] Desired_rpy
float64[3] Desired_position
float64[3] Desired_velocity
float64[3] Desired_omega
float64[3] Error_rpy
float64[3] err_int
float64 Time_Step
float64[3] Quat_error



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



(provide :space_cobot_mpc_controller/pwm_values "da432c94c450f85a4b2d4b18064bf11c")


