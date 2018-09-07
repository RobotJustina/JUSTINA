;********************************************************
;*                                                      *
;*      actions_iros.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      3/09/2018                      *
;*                                                      *
;********************************************************

(defrule task_get_object_many_locations
	?f <- (task ?plan get_object_many_rooms ?obj ?step)
	?f1 <- (item (name ?obj))
	=>
	(retract ?f)
	(printout t "Get object from many locations" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?obj status finded_and_grasped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pgetobj_many_locs ?obj ?step))
	(modify ?f1 (status nil))
	
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_get_obj_many_locs
	?goal <- (objetive get_object_many_locations ?name ?obj ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 6 11)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions set_param_in_plan ?obj)(actions_num_params 3 3 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place ) (duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions repeat_task ?name ?obj finded)(actions_num_params 2 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name ?obj finded)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions move manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status ?obj finded_and_grasped) (duration 6000)))
	(assert (finish-planner ?name 8))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_get_object_many_locations
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgetobj_many_locs ?obj ?step)
	=>
	(retract ?f1)
	(assert (objetive get_object_many_locations task_get_object_many_locations ?obj ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
