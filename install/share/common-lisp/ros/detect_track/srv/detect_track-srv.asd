
(cl:in-package :asdf)

(defsystem "detect_track-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControlDetection" :depends-on ("_package_ControlDetection"))
    (:file "_package_ControlDetection" :depends-on ("_package"))
  ))