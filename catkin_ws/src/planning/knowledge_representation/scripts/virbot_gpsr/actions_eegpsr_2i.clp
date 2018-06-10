;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	9/Junio/2018			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	EEGPSR				;;;
;;;	category 2			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule task_greet_person_no_locatio_no_locationn
	?f <- (task ?plan greet_person_no_location ?ppl ?peopleDsc ?step)
	?f1 <- (item (name question_1))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_no_location ?ppl ?peopleDsc ?step))
	(modify ?f1 (status nil))
)

(defrule task_greet_known_name 
	?f <- (task ?plan greet_known_name ?ppl ?place ?step)
	?f1 <- (item (name question_1))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_known_name ?ppl ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_offer_eat_drink_known_person
	?f <- (task ?plan offer_eat_drink_known_person ?ppl ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer_known_person ?ppl ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_follow_known_person
	?f <- (task ?plan follow_known_person ?ppl ?place ?step)
	?f1 <- (item (name man))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_known_person ?ppl ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_guide_known_person 
	?f <- (task ?plan guide_known_person ?ppl ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_known_person ?ppl ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_greet-person-dsc 
	?goal <- (objetive greet_person ?name ?ppl ?peopleDsc ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl ))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 5 9)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl)(actions_num_params 4 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 12)))
	(assert (plan (name ?name) (number 4)(actions go_to_place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-person person)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name)(actions_num_params 8 9)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions repeat_task ?name ?ppl find)(actions_num_params 5 9 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 12)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions set_plan_status ?name)(actions_num_params 5 9)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 14))
)

(defrule plan_greet-known-name 
	?goal <- (objetive greet_known_name ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl )) ;; falta la descripcion
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 5 9)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions get_person_description ?place)(actions_num_params 5 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 12)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person ?ppl)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name)(actions_num_params 8 9)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions repeat_task ?name ?ppl find)(actions_num_params 5 9 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 12)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions set_plan_status ?name)(actions_num_params 5 9)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 14))
)

(defrule plan_offer_eat_drink_known_person
	?goal <- (objetive offer_eat_drink_known_person ?name ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for" ?ppl ",_tell_me_are_you_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 5 11) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions get_person_description ?place)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 13)))
	(assert (plan (name ?name) (number 4) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions get_amount_people ?place) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions find-endurance-person ?ppl) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions make_task ?name) (actions_num_params 9 11) (duration 6000)))
	(assert (plan (name ?name) (number 9) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions make_task ?name) (actions_num_params 11 11) (duration 6000)))
	(assert (plan (name ?name) (number 11) (actions ask_and_offer ?ppl ?eatdrink ?place)))
	(assert (plan (name ?name) (number 12) (actions repeat_task ?name ?ppl offer) (actions_num_params 5 11 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions set_plan_status ?name) (actions_num_params 5 11) (duration 6000)))
	(assert (plan (name ?name) (number 13) (actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 15) (actions speech-order) (duration 6000)))
	(assert (plan (name ?name) (number 16) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 16))
)

(defrule plan_follow_person
	?goal <- (objetive follow_known_person ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 5 9) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions get_person_description ?place)(actions_num_params 5 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 12)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person ?ppl)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name)(actions_num_params 8 9)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions repeat_task ?name ?ppl find)(actions_num_params 5 9 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 12)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions find-object-man man no_location)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions set_plan_status ?name) (actions_num_params 5 9) (duration 6000)))
	(assert (plan (name ?name) (number 14)(actions update_status man followed_person)(duration 6000)))
	(assert (finish-planner ?name 14))
)

(defrule plan_guide-known-person
	?goal <- (objetive guide_known_person ?name ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 5 9) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions get_person_description ?place1)(actions_num_params 5 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 12)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person ?ppl)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name)(actions_num_params 8 9)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions repeat_task ?name ?ppl finded)(actions_num_params 5 9 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 12)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions find-object-man man_guide ?place2)))
	(assert (plan (name ?name) (number 13)(actions set_plan_status ?name)(actions_num_params 5 9) (duration 6000)))
	(assert (plan (name ?name) (number 14)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 14))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_greet-known-name  
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_no_location ?ppl ?peopleDsc ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_no_location task_greet_no_location ?ppl ?peopleDsc ?step))
)

(defrule exe_greet-known-name  
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_known_name ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_known_name task_greet_known_name ?ppl ?place ?step))
)

(defrule exe_offer_eat_drink_known_person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer_known_person ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink_known_person task_offer_eat_drink_known_person ?ppl ?eatdrink ?place ?step))
)

(defrule exe_follow-known-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_known_person ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_known_person task_follow_known_person ?ppl ?place ?step))
)

(defrule exe_guide-known-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_known_person ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_known_person task_guide_known_person ?ppl ?place1 ?place2 ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

