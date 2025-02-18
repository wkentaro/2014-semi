#!/usr/bin/env roseus
;; vim: set ft=lisp:

(require "package://jsk_2015_05_baxter_apc/euslisp/jsk_2015_05_baxter_apc/baxter-interface.l")

(ros::roseus "untuck_pose")

(setq l/r (car (last lisp::*eustop-argument*)))
(setq arm
      (cond
        ((string= l/r "left") :larm)
        ((string= l/r "right") :rarm)
        (t :arms)
        ))

(jsk_2015_05_baxter_apc::baxter-init)
(send *ri* :angle-vector (send *baxter* :untuck-pose arm) 3000)
(send *ri* :wait-interpolation)
(exit)
