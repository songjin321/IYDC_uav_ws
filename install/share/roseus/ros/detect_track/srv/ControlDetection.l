;; Auto-generated. Do not edit!


(when (boundp 'detect_track::ControlDetection)
  (if (not (find-package "DETECT_TRACK"))
    (make-package "DETECT_TRACK"))
  (shadow 'ControlDetection (find-package "DETECT_TRACK")))
(unless (find-package "DETECT_TRACK::CONTROLDETECTION")
  (make-package "DETECT_TRACK::CONTROLDETECTION"))
(unless (find-package "DETECT_TRACK::CONTROLDETECTIONREQUEST")
  (make-package "DETECT_TRACK::CONTROLDETECTIONREQUEST"))
(unless (find-package "DETECT_TRACK::CONTROLDETECTIONRESPONSE")
  (make-package "DETECT_TRACK::CONTROLDETECTIONRESPONSE"))

(in-package "ROS")





(defclass detect_track::ControlDetectionRequest
  :super ros::object
  :slots (_ControlType ))

(defmethod detect_track::ControlDetectionRequest
  (:init
   (&key
    ((:ControlType __ControlType) 0)
    )
   (send-super :init)
   (setq _ControlType (round __ControlType))
   self)
  (:ControlType
   (&optional __ControlType)
   (if __ControlType (setq _ControlType __ControlType)) _ControlType)
  (:serialization-length
   ()
   (+
    ;; int8 _ControlType
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _ControlType
       (write-byte _ControlType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _ControlType
     (setq _ControlType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ControlType 127) (setq _ControlType (- _ControlType 256)))
   ;;
   self)
  )

(defclass detect_track::ControlDetectionResponse
  :super ros::object
  :slots (_isOk ))

(defmethod detect_track::ControlDetectionResponse
  (:init
   (&key
    ((:isOk __isOk) nil)
    )
   (send-super :init)
   (setq _isOk __isOk)
   self)
  (:isOk
   (&optional __isOk)
   (if __isOk (setq _isOk __isOk)) _isOk)
  (:serialization-length
   ()
   (+
    ;; bool _isOk
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isOk
       (if _isOk (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isOk
     (setq _isOk (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass detect_track::ControlDetection
  :super ros::object
  :slots ())

(setf (get detect_track::ControlDetection :md5sum-) "b418a6e2e6ff2264b54f7352bb0ed12b")
(setf (get detect_track::ControlDetection :datatype-) "detect_track/ControlDetection")
(setf (get detect_track::ControlDetection :request) detect_track::ControlDetectionRequest)
(setf (get detect_track::ControlDetection :response) detect_track::ControlDetectionResponse)

(defmethod detect_track::ControlDetectionRequest
  (:response () (instance detect_track::ControlDetectionResponse :init)))

(setf (get detect_track::ControlDetectionRequest :md5sum-) "b418a6e2e6ff2264b54f7352bb0ed12b")
(setf (get detect_track::ControlDetectionRequest :datatype-) "detect_track/ControlDetectionRequest")
(setf (get detect_track::ControlDetectionRequest :definition-)
      "int8 ControlType
---
bool isOk

")

(setf (get detect_track::ControlDetectionResponse :md5sum-) "b418a6e2e6ff2264b54f7352bb0ed12b")
(setf (get detect_track::ControlDetectionResponse :datatype-) "detect_track/ControlDetectionResponse")
(setf (get detect_track::ControlDetectionResponse :definition-)
      "int8 ControlType
---
bool isOk

")



(provide :detect_track/ControlDetection "b418a6e2e6ff2264b54f7352bb0ed12b")


