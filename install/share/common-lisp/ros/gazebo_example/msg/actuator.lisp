; Auto-generated. Do not edit!


(cl:in-package gazebo_example-msg)


;//! \htmlinclude actuator.msg.html

(cl:defclass <actuator> (roslisp-msg-protocol:ros-message)
  ((u1
    :reader u1
    :initarg :u1
    :type cl:float
    :initform 0.0)
   (u2
    :reader u2
    :initarg :u2
    :type cl:float
    :initform 0.0)
   (u3
    :reader u3
    :initarg :u3
    :type cl:float
    :initform 0.0)
   (u4
    :reader u4
    :initarg :u4
    :type cl:float
    :initform 0.0))
)

(cl:defclass actuator (<actuator>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <actuator>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'actuator)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_example-msg:<actuator> is deprecated: use gazebo_example-msg:actuator instead.")))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_example-msg:u1-val is deprecated.  Use gazebo_example-msg:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'u2-val :lambda-list '(m))
(cl:defmethod u2-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_example-msg:u2-val is deprecated.  Use gazebo_example-msg:u2 instead.")
  (u2 m))

(cl:ensure-generic-function 'u3-val :lambda-list '(m))
(cl:defmethod u3-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_example-msg:u3-val is deprecated.  Use gazebo_example-msg:u3 instead.")
  (u3 m))

(cl:ensure-generic-function 'u4-val :lambda-list '(m))
(cl:defmethod u4-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_example-msg:u4-val is deprecated.  Use gazebo_example-msg:u4 instead.")
  (u4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <actuator>) ostream)
  "Serializes a message object of type '<actuator>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <actuator>) istream)
  "Deserializes a message object of type '<actuator>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u4) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<actuator>)))
  "Returns string type for a message object of type '<actuator>"
  "gazebo_example/actuator")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'actuator)))
  "Returns string type for a message object of type 'actuator"
  "gazebo_example/actuator")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<actuator>)))
  "Returns md5sum for a message object of type '<actuator>"
  "278bbb64bccc0a26b221d16071445863")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'actuator)))
  "Returns md5sum for a message object of type 'actuator"
  "278bbb64bccc0a26b221d16071445863")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<actuator>)))
  "Returns full string definition for message of type '<actuator>"
  (cl:format cl:nil "float64 u1~%float64 u2~%float64 u3~%float64 u4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'actuator)))
  "Returns full string definition for message of type 'actuator"
  (cl:format cl:nil "float64 u1~%float64 u2~%float64 u3~%float64 u4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <actuator>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <actuator>))
  "Converts a ROS message object to a list"
  (cl:list 'actuator
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':u2 (u2 msg))
    (cl:cons ':u3 (u3 msg))
    (cl:cons ':u4 (u4 msg))
))
