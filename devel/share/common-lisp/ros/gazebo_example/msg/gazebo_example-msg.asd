
(cl:in-package :asdf)

(defsystem "gazebo_example-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "actuator" :depends-on ("_package_actuator"))
    (:file "_package_actuator" :depends-on ("_package"))
  ))