; Auto-generated. Do not edit!


(cl:in-package detect_track-srv)


;//! \htmlinclude ControlDetection-request.msg.html

(cl:defclass <ControlDetection-request> (roslisp-msg-protocol:ros-message)
  ((ControlType
    :reader ControlType
    :initarg :ControlType
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControlDetection-request (<ControlDetection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlDetection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlDetection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detect_track-srv:<ControlDetection-request> is deprecated: use detect_track-srv:ControlDetection-request instead.")))

(cl:ensure-generic-function 'ControlType-val :lambda-list '(m))
(cl:defmethod ControlType-val ((m <ControlDetection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detect_track-srv:ControlType-val is deprecated.  Use detect_track-srv:ControlType instead.")
  (ControlType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlDetection-request>) ostream)
  "Serializes a message object of type '<ControlDetection-request>"
  (cl:let* ((signed (cl:slot-value msg 'ControlType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlDetection-request>) istream)
  "Deserializes a message object of type '<ControlDetection-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ControlType) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlDetection-request>)))
  "Returns string type for a service object of type '<ControlDetection-request>"
  "detect_track/ControlDetectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlDetection-request)))
  "Returns string type for a service object of type 'ControlDetection-request"
  "detect_track/ControlDetectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlDetection-request>)))
  "Returns md5sum for a message object of type '<ControlDetection-request>"
  "b418a6e2e6ff2264b54f7352bb0ed12b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlDetection-request)))
  "Returns md5sum for a message object of type 'ControlDetection-request"
  "b418a6e2e6ff2264b54f7352bb0ed12b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlDetection-request>)))
  "Returns full string definition for message of type '<ControlDetection-request>"
  (cl:format cl:nil "int8 ControlType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlDetection-request)))
  "Returns full string definition for message of type 'ControlDetection-request"
  (cl:format cl:nil "int8 ControlType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlDetection-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlDetection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlDetection-request
    (cl:cons ':ControlType (ControlType msg))
))
;//! \htmlinclude ControlDetection-response.msg.html

(cl:defclass <ControlDetection-response> (roslisp-msg-protocol:ros-message)
  ((isOk
    :reader isOk
    :initarg :isOk
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ControlDetection-response (<ControlDetection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlDetection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlDetection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detect_track-srv:<ControlDetection-response> is deprecated: use detect_track-srv:ControlDetection-response instead.")))

(cl:ensure-generic-function 'isOk-val :lambda-list '(m))
(cl:defmethod isOk-val ((m <ControlDetection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detect_track-srv:isOk-val is deprecated.  Use detect_track-srv:isOk instead.")
  (isOk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlDetection-response>) ostream)
  "Serializes a message object of type '<ControlDetection-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isOk) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlDetection-response>) istream)
  "Deserializes a message object of type '<ControlDetection-response>"
    (cl:setf (cl:slot-value msg 'isOk) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlDetection-response>)))
  "Returns string type for a service object of type '<ControlDetection-response>"
  "detect_track/ControlDetectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlDetection-response)))
  "Returns string type for a service object of type 'ControlDetection-response"
  "detect_track/ControlDetectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlDetection-response>)))
  "Returns md5sum for a message object of type '<ControlDetection-response>"
  "b418a6e2e6ff2264b54f7352bb0ed12b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlDetection-response)))
  "Returns md5sum for a message object of type 'ControlDetection-response"
  "b418a6e2e6ff2264b54f7352bb0ed12b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlDetection-response>)))
  "Returns full string definition for message of type '<ControlDetection-response>"
  (cl:format cl:nil "bool isOk~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlDetection-response)))
  "Returns full string definition for message of type 'ControlDetection-response"
  (cl:format cl:nil "bool isOk~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlDetection-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlDetection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlDetection-response
    (cl:cons ':isOk (isOk msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlDetection)))
  'ControlDetection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlDetection)))
  'ControlDetection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlDetection)))
  "Returns string type for a service object of type '<ControlDetection>"
  "detect_track/ControlDetection")