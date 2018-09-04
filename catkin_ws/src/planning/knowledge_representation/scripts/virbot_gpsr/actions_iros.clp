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
        (assert (condition (conditional if) (arguments ?param1 status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pgetobj_many_locs ?obj ?step))
	(modify ?f1 (status nil))
	
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_get_obj_many_locs
	?goal <- (objetive get_object_many_locations ?name ?obj ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name person finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object-man man no_location)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions update_status man followed_person)(duration 6000)))
	(assert (finish-planner ?name 6))
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
