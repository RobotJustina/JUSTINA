;********************************************************
;*                                                      *
;*      actions_kdb.clp                                 *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      11/Enero/2019                   *
;*                                                      *
;********************************************************


(defrule task_insert_kdb
	?f <- (task ?plan set_object_location ?obj ?location ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Set object location in the kdb" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments finish_objetive status set_object_location)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pset_obj_location ?obj ?location ?step))
	(modify ?f1 (status nil))
)

(defrule task_get_object_location_kdb
	?f <- (task ?plan get_object_location ?obj ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Get object location in the kdb" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments finish_objetive status get_object_location)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_obj_location ?obj ?step))
	(modify ?f1 (status nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; split the task
(defrule plan_set_object_location
	?goal <- (objetive set_object_location ?name ?obj ?location ?step)
	=>
	(retract ?goal)
	(printout t "Preueba Nuevo PLAN Set object Location in KDB")
	(assert (plan (name ?name) (number 1)(actions update_status speech nil) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions set_object_location ?obj ?location)(duration 6000)))
        (assert (plan (name ?name) (number 3)(actions speech_generator speech)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions update_status finish_objetive set_object_location) (duration 6000)))
	(assert (finish-planner ?name 4))
)

(defrule plan_get_object_location
	?goal <- (objetive get_object_location ?name ?obj ?step)
	=>
	(retract ?goal)
	(printout t "Preueba Nuevo PLAN Get object Location in KDB")
	(assert (plan (name ?name) (number 1)(actions update_status speech nil) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions get_object_location ?obj)(duration 6000)))
        (assert (plan (name ?name) (number 3)(actions speech_generator speech)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions update_status finish_objetive get_object_location) (duration 6000)))
	(assert (finish-planner ?name 4))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_set_object_location
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pset_obj_location ?obj ?location ?step)
	=>
	(retract ?f1)
	(assert (objetive set_object_location task_set_object_location ?obj ?location ?step))
)

(defrule exe_sget_object_location
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_obj_location ?obj ?step)
	=>
	(retract ?f1)
	(assert (objetive get_object_location task_get_object_location ?obj ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
