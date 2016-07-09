#!/usr/bin/env roseus
;; vim: set ft=lisp:

(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l")
(require "package://jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/util.l")

(ros::load-ros-manifest "jsk_2016_01_baxter_apc")


;; ------------------------------------------------------------------------------------------------
;; utils
;; ------------------------------------------------------------------------------------------------

(defun polygon->center (polygon-msg)
  (send
    (instance pointcloud :init :points
              (mapcar #'(lambda (p)
                          (float-vector (send p :x) (send p :y) (send p :z)))
                      (send polygon-msg :polygon :points))) :centroid))

;; ------------------------------------------------------------------------------------------------
;; collector class
;; ------------------------------------------------------------------------------------------------

(defclass in-hand-data-collection
  :super propertied-object
  :slots
  (
   arm_
   object-id_
   object-list_
   save_service_client_
   ; grasping-log-filename_
   )
  )

(defmethod in-hand-data-collection
  (:init
    ()
    (if (string= (ros::get-param "~arm") "left")
      (setq arm_ :larm)
      (setq arm_ :rarm))
    (setq object-list_ (ros::get-param "~object_list"))
    (if (eq arm_ :larm)
      (jsk_2016_01_baxter_apc::baxter-init :ctype :larm-controller :check-self-collision t)
      (jsk_2016_01_baxter_apc::baxter-init :ctype :rarm-controller :check-self-collision t))
    ;; Initialize grasping log file
    ; (setq grasping-log-filename_ (pathname (ros::get-param "~grasping_log_filename")))
    ; (with-open-file
    ;   (f grasping-log-filename_ :direction :output :if-exists :append :if-does-not-exist :create)
    ;   (format f "timestamp,object_name,grasp_success~%"))
    (setq *robot* *baxter*)
    ;; Set slots
    (setq object-id_ 0)
    (setq object-pos_ (float-vector 750 -798 -180))
    ;; Wait for other nodes
    (ros::wait-for-service "/rviz/yes_no_button")
    (ros::wait-for-service (format nil "~a_image_publisher/set_parameters" (arm-to-str arm_)))
    (ros::wait-for-service (format nil "data_collection_server_~a/save_request" (arm-to-str arm_)))
    )
  (:wait-for-user-input
    ()
    (ros::ros-info-blue "[~a] Waiting for user input to start" (ros::get-name))
    (let (yes)
      (while
        (not yes)
        (setq yes (send (ros::service-call "/rviz/yes_no_button" (instance jsk_gui_msgs::YesNoRequest)) :yes))
        )
      )
    )
  (:send-save-request
    ()
    (ros::ros-info-blue "[~a] Sending save request to data saving server" (ros::get-name))
    (let ((req (instance std_srvs::TriggerRequest :init)))
      (while
        (not
          (send
            (ros::service-call (format nil "data_collection_server_~a/save_request"
                                       (arm-to-str arm_)) req)
            :success))
        (unix::sleep 1)
        )
      )
    )
  (:go-view-hand-pose
    (&key (random nil))
    (let* ((camera-frame "kinect2_torso_rgb_optical_frame")
           (base->kinect2 (send (*ri* . _tfl) :lookup-transform "base" camera-frame (ros::time 0)))
           (tc (send (send base->kinect2 :copy-worldcoords) :translate (float-vector 0 0 800)))
           (mt (send *baxter* arm_ :end-coords))
           (av))
      (when
        (or
          (> (norm (v- (send tc :worldpos) (send mt :worldpos))) 500)  ;; far from view point
          (< (send *baxter* :left_s0 :joint-angle) -50)  ;; to avoid collision with kinect2
          (> (send *baxter* :right_s0 :joint-angle) 50)  ;; to avoid collision with kinect2
          (< (norm (v- (send base->kinect2 :worldpos) (send mt :worldpos))) 650)  ;; too close to camera
          )
        (send *baxter* arm_ :inverse-kinematics tc :use-gripper t :rotation-axis nil)
        (setq av
              (send *baxter* arm_ :inverse-kinematics tc
                    :use-gripper t
                    :rotation-axis :z
                    :thre 200 :rthre (deg2rad 60))
              )
        )
      (when
        random
        (let* ((kinect2->base (send base->kinect2 :inverse-transformation))
               (kinect2->end
                 (send (send kinect2->base :copy-worldcoords)
                       :transform (send *baxter* arm_ :end-coords :worldcoords)))
               (tc))
          (setq tc (send kinect2->end :copy-worldcoords))
          (send tc :rotate (* (- (random 6) 3) (/ 2pi 36)) :x)
          (send tc :rotate (* (- (random 6) 3) (/ 2pi 36)) :y)
          (send tc :rotate (* (- (random 6) 3) (/ 2pi 36)) :z)
          (setq tc (send (send base->kinect2 :copy-worldcoords) :transform tc))
          (setq av
            (send *baxter* arm_ :inverse-kinematics
                  tc
                  ;; fit the rotation
                  ; :use-gripper t :translation-axis :xy :thre 200 :rthre (deg2rad 60)
                  :use-gripper t :rotation-axis t :translation-axis t
                  :thre 500 :rthre (/ 2pi 36)
                  ;; second task with null-space, fit the position
                  :additional-check #'(lambda ()
                                        (and
                                          (> 1 (norm (send mt :difference-position tc)))
                                          (> (deg2rad 1) (norm (send mt :difference-rotation tc)))
                                          ))
                  :debug-view nil
                  )
            )
          )
        )
      (when
        (or
          (> (norm (v- (send tc :worldpos) (send mt :worldpos))) 500)  ;; far from view point
          (< (send *baxter* :left_s0 :joint-angle) -50)  ;; to avoid collision with kinect2
          (> (send *baxter* :right_s0 :joint-angle) 50)  ;; to avoid collision with kinect2
          (< (norm (v- (send base->kinect2 :worldpos) (send mt :worldpos))) 650)  ;; too close to camera
          )
        (ros::ros-warn "[~a] Set IK result as fail because of some conditions specified in view hand pose"
                        (ros::get-name))
        (setq av nil)
        )
      (if av
        (progn
          (send *ri* :angle-vector-sequence (list av) :fast nil 0 :scale 3.0)
          (send *ri* :wait-interpolation)
          )
        (progn
          (ros::ros-warn "[~a] Failed IK for view-hand-pose. random: ~a" (ros::get-name) random)
          (return-from :go-view-hand-pose nil)
          )
        )
      )
    (unix::sleep 1)
    )
  (:move-arm-to-view-table
    ()
    (if (eq arm_ :larm)
      (send *baxter* :larm :angle-vector
            #f(9.25049 42.3633 -132.869 125.244 140.471 107.71 -19.1602 90.0))
      (send *baxter* :rarm :angle-vector
            #f(-9.25049 42.3633 132.869 125.244 -140.471 107.71 19.1602 90.0))
      )
    (let ((random-x 100)
          (random-y 100)
          (random-z 100))
      (send *baxter* arm_ :move-end-pos
            (float-vector (random random-x) (random random-y) (random random-z))
            :local
            :use-gripper t
            :revert-if-fail nil)
      )
    (send *ri* :gripper-servo-on arm_)
    (send *ri* :angle-vector-sequence (list (send *baxter* :angle-vector)) :fast nil 0 :scale 3.0)
    (send *ri* :wait-interpolation)
    )
  (:detect-object-on-table
    ()
    (send self :move-arm-to-view-table)
    (unix::sleep 1)
    (let* ((msg) (frame_id) (pose))
      (setq msg (one-shot-subscribe
                  (format nil
                          "cpi_decomposer_~a_table/centroid_pose_array"
                          (arm-to-str arm_))
                  geometry_msgs::PoseArray
                  :after-stamp (ros::time-now)))
      (if (or (null msg) (null (send msg :poses)))
        (progn
          (ros::ros-warn "[~a] No object is found on table. Retrying.." (ros::get-name))
          (send self :detect-object-on-table)
          )
        (progn
          (setq frame_id (send msg :header :frame_id))
          (setq pose (elt (send msg :poses) 0))
          (make-coords
            :pos (send (send *ri* :tf-pose->coords frame_id pose) :worldpos))))
      ))
  (:pick-object-from-table
    ()
    (ros::ros-info-blue "[~a] Picking object from table" (ros::get-name))
    (let ((av0) (av1) (centroid (send self :detect-object-on-table)))
      (send *ri* :gripper-servo-on arm_)
      (setq av0 (send *baxter* arm_
                     :inverse-kinematics
                     (send (send centroid :copy-worldcoords)
                           :translate (float-vector 0 0 230) :world)
                     :use-gripper t
                     :rotation-axis nil))
      (unless av0 (return-from :pick-object-from-table))
      (send *ri* :angle-vector-sequence (list av0) :fast nil 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      (setq av1
            (send *baxter* arm_ :inverse-kinematics
                  (send (send centroid :copy-worldcoords)
                        :translate (float-vector 0 0 -50) :world)
                  :use-gripper t
                  :rotation-axis :z))
      (unless av1 (return-from :pick-object-from-table))
      (send *ri* :angle-vector-sequence (list av1) :fast nil 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      (send *ri* :start-grasp arm_)
      (unix::sleep 2)
      (send *ri* :gripper-servo-off arm_)
      (unix::sleep 1)
      (send *ri* :angle-vector-sequence (list av0) :fast nil 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      )
    (if (send *ri* :graspingp arm_)
      (progn
        ; (send self :write-grasping-log :success t)
        (send *ri* :angle-vector-sequence
              (list (send *baxter* :fold-to-keep-object arm_))
              :fast nil 0 :scale 3.0)
        (send *ri* :wait-interpolation)
        (send *ri* :gripper-servo-on arm_)
        t)
      (progn
        ; (send self :write-grasping-log :success nil)
        (send *ri* :stop-grasp arm_)
        (unix::sleep 5)
        nil)
      )
    )
  ; (:write-grasping-log
  ;   (&key (success))
  ;   ;; ex)
  ;   ;; timestamp,object_name,grasp_success
  ;   ;; 14XXXX.XXXXX,oreo_mega_stuf,1
  ;   ;; 14XXXX.XXXXX,oreo_mega_stuf,0
  ;   (with-open-file (f grasping-log-filename_ :direction :output :if-exists :append)
  ;     (format f "~a,~a,~a~%"
  ;             (send (ros::time-now) :to-nsec)
  ;             (elt object-list_ object-id_)
  ;             (if success 1 0))
  ;     )
  ;   )
  (:detect-table-center
    ()
    (send self :move-arm-to-view-table)
    (let* ((msg)
           (center)
           (pose (instance geometry_msgs::Pose :init))
           (plane-center))
      (setq msg (one-shot-subscribe
                  (format nil
                          "multi_plane_estimate_~a_hand_camera/output_refined_polygon"
                          (arm-to-str arm_))
                  jsk_recognition_msgs::PolygonArray))
      (if (or (null msg) (null (send msg :polygons)))
        (progn
          (ros::ros-warn "[~a] No plane is found. Retrying.." (ros::get-name))
          (send self :detect-table-center))
        (progn
          (setq center (polygon->center (elt (send msg :polygons) 0)))
          (send pose :position :x (elt center 0))
          (send pose :position :y (elt center 1))
          (send pose :position :z (elt center 2))
          (send pose :orientation :w 1)
          (setq plane-center (send *ri* :tf-pose->coords (send msg :header :frame_id) pose))
          (make-coords :pos (send plane-center :worldpos))))
      ))
  (:place-object-on-table
    ()
    (ros::ros-info-blue "[~a] Placing object on table" (ros::get-name))
    (send *ri* :gripper-servo-on arm_)
    (let ((av0 (send *baxter* :rotate-gripper arm_ 0 :relative nil))
          (av1)
          (tc (send (send (send self :detect-table-center) :copy-worldcoords)
                    :translate (float-vector 0 0 150) :world)))
      (send tc :rotate (* (- (random 6) 6) (/ 2pi 36)) :x)
      (send tc :rotate (* (- (random 6) 6) (/ 2pi 36)) :y)
      (send tc :rotate (* (- (random 6) 6) (/ 2pi 36)) :z)
      (setq av1
            (send *baxter* arm_
                  :inverse-kinematics
                  tc
                  :use-gripper t
                  :thre 200 :rthre (/ 2pi 36)
                  )
            )
      (unless av1 (return-from :place-object-on-table))
      (send *ri* :angle-vector-sequence (list av0 av1) :fast nil 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      )
    (send *ri* :stop-grasp arm_)
    (unix::sleep 5)
    )
  (:collect-data-for-an-object
    ()
    (ros::ros-info-blue "[~a] Collecting data for an object" (ros::get-name))
    (dotimes (_ 9)
      ;; Check mutex before picking object
      (let ((last-logging-time (ros::time 0)))
        (while
          (ros::get-param "/in_hand_data_collection/mutex" nil)
          (send *ri* :angle-vector-sequence (list (send *baxter* :fold-pose-back arm_)) :fast nil 0 :scale 3.0)
          (send *ri* :wait-interpolation)
          (when (> (- (send (ros::time-now) :to-sec) (send last-logging-time :to-sec)) 30)
            (ros::ros-info-green "[~a] Waiting for opposite arm. /in_hand_data_collection/mutex is True." (ros::get-name))
            (setq last-logging-time (ros::time-now))
            )
          (unix::sleep 1)
          )
        )
      (ros::set-param "/in_hand_data_collection/mutex" t)  ;; Lock mutex
      ;; Pick object from table
      (while
        (null (send self :pick-object-from-table))
        (ros::ros-warn "[~a] Failed to pick object from table. Retrying.." (ros::get-name))
        (unix::sleep 3)
        )
      (ros::set-dynparam
        (format nil "data_collection_server_~a" (arm-to-str arm_))
        (list
          (cons "save_dir"
                (format nil "~a/.ros/jsk_2016_01_baxter_apc/in_hand_data_collection/~a_attempt" (unix::getenv "HOME") (send (ros::time-now) :to-nsec)))))
      (unix::sleep 1)
      ;; View hand in a static position at first
      (while
        (not (send self :go-view-hand-pose :random nil))
        (unix::usleep (* 50 1000)))
      (unix::sleep 1)
      (send self :set-param-hand-pose)
      (if (send *ri* :graspingp arm_)
        (ros::set-param "~object_label" (elt object-list_ object-id_))
        (ros::set-param "~object_label" "no_object")
        )
      (send self :send-save-request)
      ;; View hand randomly
      (dotimes (_ 9)
        ;; View hand pose
        (while
          (not (send self :go-view-hand-pose :random t))
          (send self :go-view-hand-pose :random nil)  ;; reset if failed
          (unix::usleep (* 50 1000)))
        (send self :set-param-hand-pose)
        ;; Send saving request
        (if (send *ri* :graspingp arm_)
          (ros::set-param "~object_label" (elt object-list_ object-id_))
          (ros::set-param "~object_label" "no_object")
          )
        (send self :send-save-request)
        )
      (ros::set-param "/in_hand_data_collection/mutex" nil)  ;; Free mutex
      (while
        (null (send self :place-object-on-table))
        (ros::ros-warn "[~a] Failed to place object on table. Retrying.." (ros::get-name))
        (unix::sleep 1)
        )
      )
    )
  (:request-next-object
    ()
    (let ((object-name (elt object-list_ object-id_)))
      (ros::ros-info-blue "[~a] Requesting next object: '~a'" (ros::get-name) object-name)
      (ros::set-param "~object_label" object-name)
      (ros::set-dynparam
        (format nil "~a_image_publisher" (arm-to-str arm_))
        (list
          (cons "file_name"
                (format nil (ros::resolve-ros-path "package://jsk_apc2016_common/models/~a/image.jpg") object-name))))
      (unix::sleep 5)
      (send self :wait-for-user-input)
      )
    )
  (:set-param-hand-pose
    ()
    (send *baxter* :angle-vector (send *ri* :state :potentio-vector))  ;; update *baxter*
    (let* ((camera-frame "kinect2_torso_rgb_optical_frame")
           (end-coords
             (send
               (send (*ri* . _tfl) :lookup-transform camera-frame "base" (ros::time 0))
               :transform (send *baxter* arm_ :end-coords :worldcoords)))
           (pos (ros::pos->tf-point (send end-coords :pos)))
           (quaternion (ros::rot->tf-quaternion (send end-coords :rot))))
      (ros::set-param "~hand_pose/frame_id" camera-frame)
      (ros::set-param "~hand_pose/position/x" (send pos :x))
      (ros::set-param "~hand_pose/position/y" (send pos :y))
      (ros::set-param "~hand_pose/position/z" (send pos :z))
      (ros::set-param "~hand_pose/orientation/x" (send quaternion :x))
      (ros::set-param "~hand_pose/orientation/y" (send quaternion :y))
      (ros::set-param "~hand_pose/orientation/z" (send quaternion :z))
      (ros::set-param "~hand_pose/orientation/w" (send quaternion :w))
      )
    )
  (:mainloop
    ()
    ;; reset-pose
    (send *ri* :gripper-servo-on arm_)
    (send *baxter* :fold-pose-back arm_)
    (send *baxter* :rotate-gripper arm_ 90 :relative nil)
    (send *ri* :angle-vector-sequence
          (list (send *baxter* :angle-vector))
          :fast nil 0 :scale 6.0)
    (send *ri* :wait-interpolation)
    (send *ri* :calib-pressure-threshold arm_)
    (while
      (ros::ok)
      ;; Show next object
      (send self :request-next-object)
      ;; Collect
      (send self :collect-data-for-an-object)
      ;; Prepare for next object
      (send *ri* :angle-vector-sequence
            (list (send *baxter* :fold-pose-back arm_))
            :fast nil 0 :scale 3.0)
      (send *ri* :wait-interpolation)
      (incf object-id_)
      )
    )
  )

;; __main__
(ros::roseus "in_hand_data_collection_main" :anonymous nil)
(setq collection (instance in-hand-data-collection :init))
(send collection :mainloop)
