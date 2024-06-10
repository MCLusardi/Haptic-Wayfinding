; Auto-generated. Do not edit!


(cl:in-package stretch_demos-msg)


;//! \htmlinclude ArucoHeadScanGoal.msg.html

(cl:defclass <ArucoHeadScanGoal> (roslisp-msg-protocol:ros-message)
  ((aruco_id
    :reader aruco_id
    :initarg :aruco_id
    :type cl:integer
    :initform 0)
   (tilt_angle
    :reader tilt_angle
    :initarg :tilt_angle
    :type cl:float
    :initform 0.0)
   (publish_to_map
    :reader publish_to_map
    :initarg :publish_to_map
    :type cl:boolean
    :initform cl:nil)
   (fill_in_blindspot_with_second_scan
    :reader fill_in_blindspot_with_second_scan
    :initarg :fill_in_blindspot_with_second_scan
    :type cl:boolean
    :initform cl:nil)
   (fast_scan
    :reader fast_scan
    :initarg :fast_scan
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArucoHeadScanGoal (<ArucoHeadScanGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArucoHeadScanGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArucoHeadScanGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stretch_demos-msg:<ArucoHeadScanGoal> is deprecated: use stretch_demos-msg:ArucoHeadScanGoal instead.")))

(cl:ensure-generic-function 'aruco_id-val :lambda-list '(m))
(cl:defmethod aruco_id-val ((m <ArucoHeadScanGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:aruco_id-val is deprecated.  Use stretch_demos-msg:aruco_id instead.")
  (aruco_id m))

(cl:ensure-generic-function 'tilt_angle-val :lambda-list '(m))
(cl:defmethod tilt_angle-val ((m <ArucoHeadScanGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:tilt_angle-val is deprecated.  Use stretch_demos-msg:tilt_angle instead.")
  (tilt_angle m))

(cl:ensure-generic-function 'publish_to_map-val :lambda-list '(m))
(cl:defmethod publish_to_map-val ((m <ArucoHeadScanGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:publish_to_map-val is deprecated.  Use stretch_demos-msg:publish_to_map instead.")
  (publish_to_map m))

(cl:ensure-generic-function 'fill_in_blindspot_with_second_scan-val :lambda-list '(m))
(cl:defmethod fill_in_blindspot_with_second_scan-val ((m <ArucoHeadScanGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:fill_in_blindspot_with_second_scan-val is deprecated.  Use stretch_demos-msg:fill_in_blindspot_with_second_scan instead.")
  (fill_in_blindspot_with_second_scan m))

(cl:ensure-generic-function 'fast_scan-val :lambda-list '(m))
(cl:defmethod fast_scan-val ((m <ArucoHeadScanGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stretch_demos-msg:fast_scan-val is deprecated.  Use stretch_demos-msg:fast_scan instead.")
  (fast_scan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArucoHeadScanGoal>) ostream)
  "Serializes a message object of type '<ArucoHeadScanGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'aruco_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'aruco_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'aruco_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'aruco_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'publish_to_map) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fill_in_blindspot_with_second_scan) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fast_scan) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArucoHeadScanGoal>) istream)
  "Deserializes a message object of type '<ArucoHeadScanGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'aruco_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'aruco_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'aruco_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'aruco_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'publish_to_map) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fill_in_blindspot_with_second_scan) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fast_scan) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArucoHeadScanGoal>)))
  "Returns string type for a message object of type '<ArucoHeadScanGoal>"
  "stretch_demos/ArucoHeadScanGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArucoHeadScanGoal)))
  "Returns string type for a message object of type 'ArucoHeadScanGoal"
  "stretch_demos/ArucoHeadScanGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArucoHeadScanGoal>)))
  "Returns md5sum for a message object of type '<ArucoHeadScanGoal>"
  "7c9f72de60496d5ea3903a808d86da58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArucoHeadScanGoal)))
  "Returns md5sum for a message object of type 'ArucoHeadScanGoal"
  "7c9f72de60496d5ea3903a808d86da58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArucoHeadScanGoal>)))
  "Returns full string definition for message of type '<ArucoHeadScanGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%~%# Specify aruco ID to look for~%uint32 aruco_id~%~%# Specify the camera tilt_angle at which to scan~%float32 tilt_angle~%~%# Publish tf relative to the map frame~%bool publish_to_map~%~%# If robot should rotate to cover the blind spot generated by the mast~%bool fill_in_blindspot_with_second_scan ~%~%# If scan should stop as soon as a matching aruco ID is found~%bool fast_scan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArucoHeadScanGoal)))
  "Returns full string definition for message of type 'ArucoHeadScanGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%~%# Specify aruco ID to look for~%uint32 aruco_id~%~%# Specify the camera tilt_angle at which to scan~%float32 tilt_angle~%~%# Publish tf relative to the map frame~%bool publish_to_map~%~%# If robot should rotate to cover the blind spot generated by the mast~%bool fill_in_blindspot_with_second_scan ~%~%# If scan should stop as soon as a matching aruco ID is found~%bool fast_scan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArucoHeadScanGoal>))
  (cl:+ 0
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArucoHeadScanGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ArucoHeadScanGoal
    (cl:cons ':aruco_id (aruco_id msg))
    (cl:cons ':tilt_angle (tilt_angle msg))
    (cl:cons ':publish_to_map (publish_to_map msg))
    (cl:cons ':fill_in_blindspot_with_second_scan (fill_in_blindspot_with_second_scan msg))
    (cl:cons ':fast_scan (fast_scan msg))
))
