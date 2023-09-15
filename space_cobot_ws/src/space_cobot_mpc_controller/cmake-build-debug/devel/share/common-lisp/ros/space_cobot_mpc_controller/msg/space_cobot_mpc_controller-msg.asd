
(cl:in-package :asdf)

(defsystem "space_cobot_mpc_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "pwm_values" :depends-on ("_package_pwm_values"))
    (:file "_package_pwm_values" :depends-on ("_package"))
  ))