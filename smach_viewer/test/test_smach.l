#!/usr/bin/env roseus

(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")

(setq count 0)
(defun func-foo (&rest args)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (&rest args)
  (format t "Execute state BAR~%")
  :outcome2)

(defun sample-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :FOO 'func-foo))
    (send sm :add-node (instance state :init :BAR 'func-bar))
    ;; goal-states are generated in this method
    (send sm :goal-state (list :outcome3))

    ;; select a node as start-node
    (send sm :start-state :FOO)
    ;; from and to nodes are selected by name or symbol
    (send sm :add-transition :FOO :BAR :outcome1)
    (send sm :add-transition :FOO :outcome3 :outcome2)
    (send sm :add-transition :BAR :FOO :outcome2)
    sm ))

(defun main ()
  (ros::roseus "smach_eus")
  (exec-state-machine (sample-smach))
  (exit)
  )

(main)
