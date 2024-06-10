; Auto-generated. Do not edit!


(cl:in-package stretch_demos-msg)


;//! \htmlinclude VisualServoFeedback.msg.html

(cl:defclass <VisualServoFeedback> (roslisp-msg-protocol:ros-message)
  ((angle_error
    :reader angle_error
    :initarg :angle_error
    :type cl:float
    :initform 0.0)
   (distance_error
    :reader distance_error
    :initarg :distance_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass VisualServoFeedback (<VisualServoFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualServoFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualServoFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stretch_demos-msg:<VisualServoFeedback> is deprecated: use stretch_demos-msg:VisualServoFeedback instead.")))

(cl:ensure-generic-function 'angle_error-val :lambda-list '(m))
(cl:defmethod angle_error-val ((m <VisualServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:angle_error-val is deprecated.  Use stretch_demos-msg:angle_error instead.")
  (angle_error m))

(cl:ensure-generic-function 'distance_error-val :lambda-list '(m))
(cl:defmethod distance_error-val ((m <VisualServoFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:distance_error-val is deprecated.  Use stretch_demos-msg:distance_error instead.")
  (distance_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualServoFeedback>) ostream)
  "Serializes a message object of type '<VisualServoFeedback>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualServoFeedback>) istream)
  "Deserializes a message object of type '<VisualServoFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualServoFeedback>)))
  "Returns string type for a message object of type '<VisualServoFeedback>"
  "stretch_demos/VisualServoFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualServoFeedback)))
  "Returns string type for a message object of type 'VisualServoFeedback"
  "stretch_demos/VisualServoFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualServoFeedback>)))
  "Returns md5sum for a message object of type '<VisualServoFeedback>"
  "b36919b18c17a48cbd98a249a4c96a61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualServoFeedback)))
  "Returns md5sum for a message object of type 'VisualServoFeedback"
  "b36919b18c17a48cbd98a249a4c96a61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualServoFeedback>)))
  "Returns full string definition for message of type '<VisualServoFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%# angle error is the angular error between source and target frames~%float64 angle_error~%~%# distance error is the euclidean error between source and target frames~%float64 distance_error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualServoFeedback)))
  "Returns full string definition for message of type 'VisualServoFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback~%# angle error is the angular error between source and target frames~%float64 angle_error~%~%# distance error is the euclidean error between source and target frames~%float64 distance_error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualServoFeedback>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualServoFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualServoFeedback
    (cl:cons ':angle_error (angle_error msg))
    (cl:cons ':distance_error (distance_error msg))
))
