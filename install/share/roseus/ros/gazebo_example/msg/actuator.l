;; Auto-generated. Do not edit!


(when (boundp 'gazebo_example::actuator)
  (if (not (find-package "GAZEBO_EXAMPLE"))
    (make-package "GAZEBO_EXAMPLE"))
  (shadow 'actuator (find-package "GAZEBO_EXAMPLE")))
(unless (find-package "GAZEBO_EXAMPLE::ACTUATOR")
  (make-package "GAZEBO_EXAMPLE::ACTUATOR"))

(in-package "ROS")
;;//! \htmlinclude actuator.msg.html


(defclass gazebo_example::actuator
  :super ros::object
  :slots (_u1 _u2 _u3 _u4 ))

(defmethod gazebo_example::actuator
  (:init
   (&key
    ((:u1 __u1) 0.0)
    ((:u2 __u2) 0.0)
    ((:u3 __u3) 0.0)
    ((:u4 __u4) 0.0)
    )
   (send-super :init)
   (setq _u1 (float __u1))
   (setq _u2 (float __u2))
   (setq _u3 (float __u3))
   (setq _u4 (float __u4))
   self)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:u2
   (&optional __u2)
   (if __u2 (setq _u2 __u2)) _u2)
  (:u3
   (&optional __u3)
   (if __u3 (setq _u3 __u3)) _u3)
  (:u4
   (&optional __u4)
   (if __u4 (setq _u4 __u4)) _u4)
  (:serialization-length
   ()
   (+
    ;; float64 _u1
    8
    ;; float64 _u2
    8
    ;; float64 _u3
    8
    ;; float64 _u4
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _u1
       (sys::poke _u1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u2
       (sys::poke _u2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u3
       (sys::poke _u3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u4
       (sys::poke _u4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _u1
     (setq _u1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u2
     (setq _u2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u3
     (setq _u3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u4
     (setq _u4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get gazebo_example::actuator :md5sum-) "278bbb64bccc0a26b221d16071445863")
(setf (get gazebo_example::actuator :datatype-) "gazebo_example/actuator")
(setf (get gazebo_example::actuator :definition-)
      "float64 u1
float64 u2
float64 u3
float64 u4

")



(provide :gazebo_example/actuator "278bbb64bccc0a26b221d16071445863")


