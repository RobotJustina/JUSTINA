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

(defrule task_get_person_many_locations
	?f <- (task ?plan find_person_in_many_rooms ?person ?step)
	?f1 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Find person in many rooms" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?person status finded_in_some_room)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfind_person_in_many_rooms ?person ?step))
	(modify ?f1 (status nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_get_obj_many_locs
	?goal <- (objetive get_object_many_locations ?name ?obj ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 2 4)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions set_param_in_plan ?obj)(actions_num_params 3 3 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place dummy_place) (duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions repeat_task ?name ?obj finded)(actions_num_params 2 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name ?obj finded)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions move manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions set_plan_status ?name)(actions_num_params 2 4)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status ?obj finded_and_grasped) (duration 6000)))
	(assert (finish-planner ?name 9))
)

(defrule plan_find_how_many_people_no_location
	?goal <- (objetive find_person_in_many_rooms ?name ?person ?step)
	=>
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 2 4)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions set_param_in_plan ?person)(actions_num_params 3 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place dummy_place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-person specific ?person dummy_place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions repeat_task ?name ?person finded) (actions_num_params 2 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions set_plan_status ?name)(actions_num_params 2 4) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?person finded_in_some_room) (duration 6000)))
	(assert (finish-planner ?name 7))
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

(defrule exe_find_person_in_many_rooms
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfind_person_in_many_rooms ?person ?step)
	=>
	(retract ?f1)
	(assert (objetive find_person_in_many_rooms task_find_person_in_many_rooms ?person ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
