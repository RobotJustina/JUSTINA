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
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_no_location ?ppl ?peopleDsc ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_greet_person_no_location_a 
	?f <- (task ?plan greet_person_no_location ?ppl ?step)
	?f1 <- (item (name question_1))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_no_location ?ppl ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_greet_known_name 
	?f <- (task ?plan greet_known_name ?ppl ?place ?step)
	?f1 <- (item (name question_1))
	?f2 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_known_name ?ppl ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_offer_eat_drink_known_person
	?f <- (task ?plan offer_eat_drink_known_person ?ppl ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	?f2 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer_known_person ?ppl ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_follow_known_person
	?f <- (task ?plan follow_known_person ?ppl ?place ?step)
	?f1 <- (item (name man))
	?f2 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_known_person ?ppl ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_guide_known_person 
	?f <- (task ?plan guide_known_person ?ppl ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_known_person ?ppl ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)
(defrule task_guide_man_no_location
	?f <- (task ?plan guide_person_no_location ?ppl ?place1 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person_no_location ?ppl ?place1 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_find_how_many_people_dsc_no_location
	?f <- (task ?plan find_how_many_people_no_location ?ppl ?peopleDsc ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfhmp_no_location ?ppl ?peopleDsc ?step))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
)

(defrule task_find_how_many_people_no_location 
	?f <- (task ?plan find_how_many_people_no_location ?ppl ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded) (true-state (+ ?step 1))(false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfhmp_no_location ?ppl ?step))
	(modify ?f1 (status nil))
)

(defrule task_offer_eat_drink_no_location
	?f <- (task ?plan offer_eat_drink_no_location ?ppl ?eatdrink ?step)
	?f1 <- (item (name offer))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer_no_location ?ppl ?eatdrink ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_offer_eat_drink_dsc_no_location
	?f <- (task ?plan offer_eat_drink_no_location ?ppl ?peopleDsc ?eatdrink ?step)
	?f1 <- (item (name offer))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer_no_location ?ppl ?peopleDsc ?eatdrink ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_guide_person_no_location
	?f <- (task ?plan guide_person_no_location ?ppl ?peopleDsc ?place1 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person_no_location ?ppl ?peopleDsc ?place1 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)
(defrule task_guide_person_outfit_no_location
	?f <- (task ?plan guide_person_no_location ?ppl ?color ?outfit ?place1 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person_outfit_no_location ?ppl ?color ?outfit ?place1 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_greet-person-no-location 
	?goal <- (objetive greet_no_location ?name ?ppl ?peopleDsc ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl ))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 6 11)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions ask_for_incomplete ?ppl origin)(actions_num_params 5 6)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)))
	(assert (plan (name ?name) (number 5)(actions go_to_place)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person person )(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name person finded)(actions_num_params 8 11)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name)(actions_num_params 10 11)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions repeat_task ?name ?ppl find)(actions_num_params 6 11 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions make_task ?name)(actions_num_params 14 14)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions set_plan_status ?name)(actions_num_params 6 11)(duration 6000)))
	(assert (plan (name ?name) (number 16)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 16))
)

(defrule plan_greet-person-no-location-a
	?goal <- (objetive greet_no_location ?name ?ppl ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(assert (plan (name ?name) (number 1)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl origin)(actions_num_params 4 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 7)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ) (duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person person)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name person finded)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 9))
)

(defrule plan_greet-known-name 
	?goal <- (objetive greet_known_name ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl )) ;; falta la descripcion
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 6 11)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions get_person_description ?ppl ?place)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)))
	(assert (plan (name ?name) (number 5)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person ?ppl)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name ?ppl finded)(actions_num_params 8 11)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name)(actions_num_params 10 11)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions repeat_task ?name ?ppl find)(actions_num_params 6 11 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions make_task ?name)(actions_num_params 14 14)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions set_plan_status ?name)(actions_num_params 6 11)(duration 6000)))
	(assert (plan (name ?name) (number 16)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 16))
)

(defrule plan_offer_eat_drink_known_person
	?goal <- (objetive offer_eat_drink_known_person ?name ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 6 14) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions get_person_description ?ppl ?place)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)))
	(assert (plan (name ?name) (number 5) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions get_amount_people ?place) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions find-endurance-person ?ppl) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions make_task ?name ?ppl finded)(actions_num_params 9 14) (duration 6000)))
	(assert (plan (name ?name) (number 9) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions make_task ?name) (actions_num_params 11 14) (duration 6000)))
	(assert (plan (name ?name) (number 11) (actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 12) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 13) (actions make_task ?name) (actions_num_params 14 14) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions ask_and_offer ?ppl ?eatdrink ?place)))
	(assert (plan (name ?name) (number 15) (actions repeat_task ?name ?ppl find) (actions_num_params 6 14 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 16) (actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 17) (actions speech-order) (duration 6000)))
	(assert (plan (name ?name) (number 18) (actions set_plan_status ?name) (actions_num_params 6 14) (duration 6000)))
	(assert (plan (name ?name) (number 19) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 19))
)

(defrule plan_follow_k_person
	?goal <- (objetive follow_known_person ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 6 11) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions get_person_description ?ppl ?place)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)))
	(assert (plan (name ?name) (number 5)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person ?ppl)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name ?ppl finded)(actions_num_params 8 11)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name)(actions_num_params 10 11)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions repeat_task ?name ?ppl find)(actions_num_params 6 11 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions make_task ?name)(actions_num_params 14 14)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions find-object-man man no_location)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions set_plan_status ?name) (actions_num_params 6 11) (duration 6000)))
	(assert (plan (name ?name) (number 16)(actions update_status man followed_person)(duration 6000)))
	(assert (finish-planner ?name 16))
)

(defrule plan_guide-known-person
	?goal <- (objetive guide_known_person ?name ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_" ?ppl ",_tell_me_are_you_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 6 11) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions get_person_description ?ppl ?place1)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)))
	(assert (plan (name ?name) (number 5)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person ?ppl)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name ?ppl finded)(actions_num_params 8 11)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name)(actions_num_params 10 11)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions repeat_task ?name ?ppl find)(actions_num_params 6 11 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions make_task ?name)(actions_num_params 14 14)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions find-object-man man_guide ?place2)))
	(assert (plan (name ?name) (number 15)(actions set_plan_status ?name)(actions_num_params 6 11) (duration 6000)))
	(assert (plan (name ?name) (number 16)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 16))
)

(defrule plan_find_how_many_people_no_location 
	?goal <- (objetive find_how_many_people_no_location ?name ?ppl ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Find how many people task" crlf)
	(bind ?speech (str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl origin)(actions_num_params 4 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 7)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions go_to_place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-many-people ?ppl)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions go_to_place current_loc) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions speech-response speech) (duration 6000)))
	(assert (plan (name ?name) (number 8)(actions set_plan_status ?name dummy) (duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 9))
)

(defrule plan_find_how_many_people_dsc_no_location
        ?goal <- (objetive find_how_many_people_no_location ?name ?ppl ?peopleDsc ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find how many people Task" crlf)
	(bind ?speech(str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl origin)(actions_num_params 4 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 7)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions go_to_place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-many-people  ?ppl ?peopleDsc)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions speech-response speech)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 9))
)

(defrule plan_offer_eat_drink_no_location
	?goal <- (objetive offer_eat_drink_no_location ?name ?ppl ?eatdrink ?step)
	=>
	(retract ?goal)
	(printout t "Preueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_offering_some_to_" ?eatdrink  "_to_the_" ?ppl ",_tell_me_are_you_a_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 6 13) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions ask_for_incomplete ?ppl origin)(actions_num_params 5 7)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 14)(duration 6000)))
	(assert (plan (name ?name) (number 5) (actions go_to_place) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions get_amount_people) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions find-endurance-person person) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions make_task ?name person finded)(actions_num_params 9 13)(duration 6000)))
	(assert (plan (name ?name) (number 9) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions make_task ?name) (actions_num_params 11 13)))
	(assert (plan (name ?name) (number 11) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 12) (actions make_task ?name) (actions_num_params 13 13) (duration 6000)))
	(assert (plan (name ?name) (number 13) (actions ask_and_offer ?ppl ?eatdrink kitchen) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions repeat_task ?name ?ppl offer) (actions_num_params 6 13 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 17) (actions set_plan_status ?name) (actions_num_params 6 13) (duration 6000)))
	(assert (plan (name ?name) (number 15) (actions go_to_place current_loc) (duration 6000)))
	(assert (plan (name ?name) (number 16) (actions speech-oreder) (duration 6000)))
	(assert (plan (name ?name) (number 18) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 18))
)

(defrule plan_offer_eat_drink_dsc_no_location
	?goal <- (objetive offer_eat_drink_no_location ?name ?ppl ?peopleDsc ?eatdrink ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_offering_some_to_" ?eatdrink "_to_the_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 6 12) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions ask_for_incomplete ?ppl origin)(actions_num_params 5 6)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name incomplete asked)(actions_num_params 5 13)(duration 6000)))
	(assert (plan (name ?name) (number 5) (actions go_to_place) (duration 6000)))
	;(assert (plan (name ?name) (number 6) (actions get_amount_people) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions find-endurance-person person ?peopleDsc) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions make_task ?name person finded)(actions_num_params 8 12) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 9) (actions make_task ?name) (actions_num_params 10 12) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 11) (actions make_task ?name) (actions_num_params 13 13) (duration 6000)))
	(assert (plan (name ?name) (number 12) (actions ask_and_offer ?ppl ?eatdrink)))
	(assert (plan (name ?name) (number 13) (actions repeat_task ?name ?ppl offer) (actions_num_params 6 12 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 16) (actions set_plan_status ?name) (actions_num_params 6 12) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 15) (actions speech-order) (duration 6000)))
	(assert (plan (name ?name) (number 17) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 17))
)

(defrule plan_guide_man_no_location 
	?goal <- (objetive guide_man_no_location ?name ?ppl ?place1 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions update_status incomplete nil)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl destiny)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 7)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person person ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name person finded)(actions_num_params 7 7)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions find-object-man man_guide) (duration 6000)))
	(assert (plan (name ?name) (number 8)(actions set_plan_status ?name dummy) (duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 9))
)

(defrule plan_guide-person-no-location 
	?goal <- (objetive guide_person_no_location ?name ?ppl ?peopleDsc ?place1 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 5 10) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl destiny)(actions_num_params 13 13)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 13)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person person ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name person finded)(actions_num_params 7 10)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 8)(actions make_task ?name)(actions_num_params 9 10)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions repeat_task ?name ?ppl find)(actions_num_params 5 10 1 3)(duration 6000)))
	;(assert (plan (name ?name) (number 11)(actions make_task ?name conf true)(actions_num_params 12 12)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions make_task ?name)(actions_num_params 13 13)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions find-object-man man_guide)))
	(assert (plan (name ?name) (number 14)(actions set_plan_status ?name)(actions_num_params 5 10) (duration 6000)))
	(assert (plan (name ?name) (number 15)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 15))
)

(defrule plan_guide-person-outfit_no_location 
	?goal <- (objetive guide_person_outfit_no_location ?name ?ppl ?color ?outfit ?place1 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?color "_" ?outfit "_" ?ppl ",_tell_me_are_you_a_" ?color "_" ?outfit "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 5 10) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions ask_for_incomplete ?ppl destiny)(actions_num_params 13 13)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name incomplete asked)(actions_num_params 4 13)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions find-endurance-person person ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name person finded)(actions_num_params 7 13)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?color ?outfit ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 8)(actions make_task ?name)(actions_num_params 9 10)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions repeat_task ?name ?ppl find)(actions_num_params 5 10 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions make_task ?name)(actions_num_params 13 13)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions find-object-man man_guide)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions set_plan_status ?name)(actions_num_params 5 10)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 15))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_greet-person-no-location-a
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_no_location ?ppl ?peopleDsc ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_no_location task_greet_no_location ?ppl ?peopleDsc ?step))
)

(defrule exe_greet-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_no_location ?ppl ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_no_location task_greet_no_location ?ppl ?step))
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

(defrule exe_find-how-many-people-no-location 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfhmp_no_location ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive find_how_many_people_no_location task_find_how_many_people_no_location ?ppl ?step))
)

(defrule exe_find-how-many-people_dsc-no-location 
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (task pfhmp_no_location ?ppl ?peopleDsc ?step)
        =>
        (retract ?f1)
        (assert (objetive find_how_many_people_no_location task_find_how_many_people_no_location ?ppl ?peopleDsc ?step))
)

(defrule exe_offer_eat_drink_no_location 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer_no_location ?ppl ?eatdrink ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink_no_location task_offer_eat_drink_no_location ?ppl ?eatdrink ?step))
)

(defrule exe_offer_eat_drink_dsc_no_location
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer_no_location ?ppl ?peopleDsc ?eatdrink ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink_no_location task_offer_eat_drink_no_location ?ppl ?peopleDsc ?eatdrink ?step))
)

(defrule exe_guide-man-no-location 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person_no_location ?ppl ?place1 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_man_no_location task_guide_man_no_location ?ppl ?place1 ?step))
)

(defrule exe_guide-person-no-location
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person_no_location ?ppl ?peopleDsc ?place1 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person_no_location task_guide_person_no_location ?ppl ?peopleDsc ?place1 ?step))
)

(defrule exe_guide_person_outfit_no_location 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person_outfit_no_location ?ppl ?color ?outfit ?place1 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person_outfit_no_location task_guide_person_outfit_no_location ?ppl ?color ?outfit ?place1 ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

