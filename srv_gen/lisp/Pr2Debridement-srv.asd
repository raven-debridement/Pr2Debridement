
(cl:in-package :asdf)

(defsystem "Pr2Debridement-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReturnJointStates" :depends-on ("_package_ReturnJointStates"))
    (:file "_package_ReturnJointStates" :depends-on ("_package"))
  ))