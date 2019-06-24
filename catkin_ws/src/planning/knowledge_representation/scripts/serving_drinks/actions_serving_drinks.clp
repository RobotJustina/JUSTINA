;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	10/Enero/2019			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	Serving Drinks			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule task_serving_drinks
	?f <- (task ?plan serving_drinks ?place1 ?place2 ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Serving dring test")
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finish_test)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pserving_drinks ?place1 ?place2 ?step))
	(modify ?f1 (status nil))	
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule plan_serving_drinks
	?goal <- (objetive serving_drinks ?name ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(printout  t "Nuevo plan Serving Drinks")
	(bind ?what(str-cat "Hello_what_is_your_name"))
	(bind ?remember (str-cat "Please look at me, I try to remember you"))
	(bind ?confirmation (str-cat "Hello my name is justina, Do_you_want_some_to_drink"))

	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 4 16)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions go_to_place ?place1)(duration 6000)))
        (assert (plan (name ?name) (number 3)(actions find-objects-on-location ?place1)(duration 6000)))
        (assert (plan (name ?name) (number 4)(actions go_to_place ?place2)(duration 6000)))

        (assert (plan (name ?name) (number 5)(actions find-person person ?place2)(duration 6000)))
        (assert (plan (name ?name) (number 6) (actions make_task ?name person went)(actions_num_params 7 11) (duration 6000)))
        (assert (plan (name ?name) (number 7) (actions confirmation ?confirmation) (duration 6000)))
        (assert (plan (name ?name) (number 8) (actions make_task ?name) (actions_num_params 9 11)))
        (assert (plan (name ?name) (number 9) (actions offer_drink drinks) (actions_num_params 2) (duration 6000)));;offer one drink
        (assert (plan (name ?name) (number 10) (actions make_task ?name offer offered)(actions_num_params 11 11)(duration 6000)))
        (assert (plan (name ?name) (number 11) (actions train_person)(duration 6000)))
        (assert (plan (name ?name) (number 12) (actions repeat_task ?name people last_offer) (actions_num_params 5 11 1 5) (duration 6000)))
        ;(assert (plan (name ?name) (number 13) (actions set_plan_status ?name)(actions_num_params 4 11)(duration 6000)))

        ;(assert (plan (name ?name) (number 14)(actions set_plan_status ?name)(actions_num_params 15 17)(duration 6000)))
        (assert (plan (name ?name) (number 13)(actions make_task ?name offer offered) (actions_num_params 14 16)))
        (assert (plan (name ?name) (number 14) (actions get_ordered_objects ?place1)(actions_num_params 1 1)(duration 6000)));;;(actions_params obj_per_attemp total_orders)
        ;(assert (plan (name ?name) (number 16) (actions find-person specific)(actions_num_params 1 2)(duration 6000)))
        (assert (plan (name ?name) (number 15) (actions deliver_order ?place2) (duration 6000)))
        (assert (plan (name ?name) (number 16) (actions repeat_task ?name people deliver_order) (actions_num_params 13 15 1 5) (duration 6000)))
        ;(assert (plan (name ?name) (number 18) (actions set_plan_status ?name)(actions_num_params 15 16)(duration 6000)))
        ;(assert (plan (name ?name) (number 19) (actions set_plan_status ?name)(actions_num_params 4 16)(duration 6000)))
        (assert (plan (name ?name) (number 17) (actions repeat_task ?name people dummy_state) (actions_num_params 4 16 1 2) (duration 6000)))
        (assert (plan (name ?name) (number 18) (actions set_plan_status ?name)(actions_num_params 4 16)(duration 6000)))
        (assert (plan (name ?name) (number 19) (actions update_status finish_objetive finish_test)(duration 6000)))

	(assert (finish-planner ?name 19))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe_serving_drinks
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pserving_drinks ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive serving_drinks task_serving_drinks ?place1 ?place2 ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
