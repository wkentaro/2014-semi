;; vim: set ft=lisp:


(load "package://jsk_2015_05_baxter_apc/euslisp/jsk_2015_05_baxter_apc/baxter-interface.l")
(load "package://jsk_2015_05_baxter_apc/euslisp/jsk_2015_05_baxter_apc/reachable-space.l")


(defun main ()
  (setq reachable-space (compute-reachable-space :draw nil))
  (visualize-reachable-space reachable-space)
  )


(setq *baxter* (instance jsk_2015_05_baxter_apc::baxter-robot :init))
(setq *robot* *baxter*)
(main)
