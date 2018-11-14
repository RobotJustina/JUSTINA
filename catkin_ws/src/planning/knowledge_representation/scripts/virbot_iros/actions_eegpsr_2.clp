;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	18/Abril/2018			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	EEGPSR				;;;
;;;	category 2			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;#tell me how many $people there are in the $room
;#tell me how many $peopleR in the $room
;#tell me how many $ppl in the $room are $peopleDsc
(defrule task_find_how_many_people_dsc
	?f <- (task ?plan find_how_many_people ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfhmp) (actor robot)(obj ?ppl)(from ?peopleDsc)(to ?place)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
)

(defrule task_find_how_many_people 
	?f <- (task ?plan find_how_many_people ?ppl ?place ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded) (true-state (+ ?step 1))(false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfhmp ?ppl ?place ?step))
	(modify ?f1 (status nil))
)

;#offer something to (eat | drink) to all the $people in the $room
(defrule task_offer_eat_drink 
	?f <- (task ?plan offer_eat_drink ?ppl ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer ?ppl ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_offer_eat_drink_dsc
	?f <- (task ?plan offer_eat_drink ?ppl ?peopleDsc ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer ?ppl ?peopleDsc ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

;#bring the {objetc} to the $person in the $room
(defrule task_bring_obj_to_person
	?f <- (task ?plan bring_obj_to_prsn ?ppl ?obj ?place ?step)
	?f1 <- (item (name ?obj))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Bring object to person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?obj status handover) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pbringtop ?ppl ?obj ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_bring_obj_to_prsn
	?f <- (task ?plan bring_obj_to_prsn ?ppl ?peopleDsc ?obj ?place ?step)
	?f1 <- (item (name ?obj))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Bring object to person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?obj status handover) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pbringtop ?ppl ?peopleDsc ?obj ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_bring_obj_to_prsn_color_outfit
	?f <- (task ?plan bring_obj_to_prsn ?ppl ?color ?outfit ?obj ?place ?step)
	?f1 <- (item (name ?obj))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Bring object to person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?obj status handover) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pbringtop ?ppl ?color ?outfit ?obj ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

;#$greet the $person in the $room
(defrule task_greet_person
	?f <- (task ?plan greet_person ?ppl ?place ?step)
	?f1 <- (item (name question_1))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet ?ppl ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_greet_person_dsc
	?f <- (task ?plan greet_person ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name question_1))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet ?ppl ?peopleDsc ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_greet_person_outfit 
	?f <- (task ?plan greet_peson ?ppl ?color ?outfit ?place ?step)
	?f1 <- (item (name question_1))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_outfit ?ppl ?color ?outfit ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)


        ;#$vbfollow the $fgwhor
        ;#$vbfollow the $fgwho $fbriefing
(defrule task_follow_man
	?f <- (task ?plan follow_person ?ppl ?place ?step)
	?f1 <- (item (name man))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person ?ppl ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_follow_person
	?f <- (task ?plan follow_person ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name man))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person ?ppl ?peopleDsc ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

        ;#$vbfollow the $fgwhor
(defrule task_follow_person_outfit
	?f <- (task ?plan follow_person ?ppl ?color ?outfit ?place ?step)
	?f1 <- (item (name man))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Follow person to room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person_outfit ?ppl ?color ?outfit ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

        ;#$vbguide the $fgwhor to the (exit | {room 2})
(defrule task_guide_man
	?f <- (task ?plan guide_person ?ppl ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person ?ppl ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_guide_person
	?f <- (task ?plan guide_person ?ppl ?peopleDsc ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person ?ppl ?peopleDsc ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_guide_person_outfit
	?f <- (task ?plan guide_person ?ppl ?color ?outfit ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed_person) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_describe_person_to_me
	?f <- (task ?plan describe_person ?posture ?place ?step)
	?f1 <- (item (name question_2))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Describe the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_2 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pdescribe_person_to_me ?posture ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_describe_person
	?f <- (task ?plan describe_person ?posture ?place1 ?place2 ?step)
	?f1 <- (item (name question_2))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Describe the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_2 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pdescribe_person ?posture ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

;$vbmeet {name} at the {beacon 1}
(defrule task_remind_person
	?f <- (task ?plan remind_person ?person ?place ?step)
	?f1 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Conoce y recuerda a una person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?person status reminded) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task premind_person ?person ?place ?step))
	(modify ?f1 (status remind))
)

(defrule task_greet_known_person
	?f <- (task ?plan greet_known_person ?place ?step)
	?f1 <- (item (name ?person) (status remind))
	=>
	(retract ?f)
	(printout t "Busca y saluda a una persona que conoces" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?person status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_known_person ?place ?step))
) 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; subtask

(defrule plan_find_how_many_people
	?goal <- (objetive find_how_many_people ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Find how many people task" crlf)
	(bind ?speech (str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-many-people ?ppl ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place current_loc) (duration 6000)))
	(assert (plan (name ?name) (number 4)(actions speech-response speech) (duration 6000)))
	(assert (plan (name ?name) (number 5)(actions set_plan_status ?name dummy) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_find_how_many_people_dsc
        ?goal <- (objetive find_how_many_people ?name ?ppl ?peopleDsc ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find how many people Task" crlf)
	(bind ?speech(str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-many-people  ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions speech-response speech)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_offer_eat_drink
	?goal <- (objetive offer_eat_drink ?name ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Preueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_offering_some_to_" ?eatdrink  "_to_the_" ?ppl ",_tell_me_are_you_a_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 3 10) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 3) (actions get_amount_people ?place) (duration 6000)))
	(assert (plan (name ?name) (number 4) (actions find-endurance-person person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions make_task ?name person finded)(actions_num_params 6 10) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions make_task ?name) (actions_num_params 8 10)))
	(assert (plan (name ?name) (number 8) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 9) (actions make_task ?name) (actions_num_params 10 10) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions ask_and_offer ?ppl ?eatdrink ?place) (duration 6000)))
	(assert (plan (name ?name) (number 11) (actions repeat_task ?name ?ppl offer) (actions_num_params 3 10 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions set_plan_status ?name) (actions_num_params 3 10) (duration 6000)))
	(assert (plan (name ?name) (number 12) (actions go_to_place current_loc) (duration 6000)))
	(assert (plan (name ?name) (number 13) (actions speech-oreder) (duration 6000)))
	(assert (plan (name ?name) (number 15) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 15))
)

(defrule plan_offer_eat_drink_dsc
	?goal <- (objetive offer_eat_drink ?name ?ppl ?peopleDsc ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Offer something to drink")
	(bind ?confirmation(str-cat "Hello_i_am_offering_some_to_" ?eatdrink "_to_the_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl))
	(bind ?confirmation1(str-cat "Do_you_want_some_to_" ?eatdrink))
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 3 10) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 3) (actions get_amount_people ?place) (duration 6000)))
	(assert (plan (name ?name) (number 4) (actions find-endurance-person person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions make_task ?name person finded)(actions_num_params 6 10)(duration 6000)))
	(assert (plan (name ?name) (number 6) (actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions make_task ?name) (actions_num_params 8 10) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions confirmation ?confirmation1) (duration 6000)))
	(assert (plan (name ?name) (number 9) (actions make_task ?name) (actions_num_params 10 10) (duration 6000)))
	(assert (plan (name ?name) (number 10) (actions ask_and_offer ?ppl ?eatdrink ?place)))
	(assert (plan (name ?name) (number 11) (actions repeat_task ?name ?ppl offer) (actions_num_params 3 10 1 3) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions set_plan_status ?name) (actions_num_params 3 10) (duration 6000)))
	(assert (plan (name ?name) (number 12) (actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 13) (actions speech-order) (duration 6000)))
	(assert (plan (name ?name) (number 15) (actions update_status offer final_offer) (duration 6000)))
	(assert (finish-planner ?name 15))
)

(defrule plan_bring-obj-to-person 
	?goal <- (objetive bring_obj_to_prsn ?name ?ppl ?obj ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo plan bring object to someone")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?ppl ",_tell_me_are_you_a_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 8 13) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions move_eegpsr manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name ?obj grabed)(actions_num_params 7 16)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name person ?place)(actions_num_params 10 13)(duration 6000)))
	;(assert (plan (name ?name) (number 8)(actions find-endurance-person ?ppl ?place)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name) (actions_num_params 12 13) (duration 6000)))
	(assert (plan (name ?name) (number 12)(actions update_status ?ppl find) (duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status conf true) (duration 6000)))
	(assert (plan (name ?name) (number 14)(actions repeat_task ?name ?ppl find) (actions_num_params 8 13 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions make_task ?name)(actions_num_params 16 16)(duration 6000)))
	(assert (plan (name ?name) (number 16)(actions drop person ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 17)(actions set_plan_status ?name) (actions_num_params 8 13) (duration 6000)))
	(assert (plan (name ?name) (number 18)(actions update_status ?obj handover)(duration 6000)))
	(assert (finish-planner ?name 18))
)

(defrule plan_bring-obj-to-person-dsc
	?goal <- (objetive bring_obj_to_prsn ?name ?ppl ?peopleDsc ?obj ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN bring object to someone")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl ))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 8 13) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions move_eegpsr manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name ?obj grabed)(actions_num_params 7 16)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions find-endurance-person person ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name person finded)(actions_num_params 10 13)(duration 6000)))
	;(assert (plan (name ?name) (number 8)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 13)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions repeat_task ?name ?ppl find) (actions_num_params 8 13 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions make_task ?name)(actions_num_params 16 16) (duration 6000)))
	(assert (plan (name ?name) (number 16)(actions drop person ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 17)(actions set_plan_status ?name)(actions_num_params 8 13) (duration 6000)))
	(assert (plan (name ?name) (number 18)(actions update_status ?obj handover)(duration 6000)))
	(assert (finish-planner ?name 18))
)

(defrule plan_bring-obj-to-person-color-outfit 
	?goal <- (objetive bring_obj_to_prsn ?name ?ppl ?color ?outfit ?obj ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN bring object to someone")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?color "_" ?outfit "_" ?ppl ",_tell_me_are_you_a_" ?color "_" ?outfit "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 8 13) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions move_eegpsr manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name ?obj grabed)(actions_num_params 7 16)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions make_task ?name person finded)(actions_num_params 10 13)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions confirmation ?confirmation)(duration 6000)))
	;(assert (plan (name ?name) (number 8)(actions find-endurance-person ?ppl ?color ?outfit ?place)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions make_task ?name)(actions_num_params 12 13)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 14)(actions repeat_task ?name ?ppl find) (actions_num_params 8 13 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 15)(actions make_task ?name)(actions_num_params 16 16) (duration 6000)))
	(assert (plan (name ?name) (number 16)(actions drop person ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 17)(actions set_plan_status ?name)(actions_num_params 8 13)(duration 6000)))
	(assert (plan (name ?name) (number 18)(actions update_status ?obj handover)(duration 6000)))
	(assert (finish-planner ?name 18))
)

(defrule plan_greet-person
	?goal <- (objetive greet_person ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name person finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions update_status question_1 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_greet-person-dsc 
	?goal <- (objetive greet_person ?name ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl ))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8)(duration 6000)))	
	(assert (plan (name ?name) (number 2)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions set_plan_status ?name)(actions_num_params 3 8)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_greet-person-outfit
	?goal <- (objetive greet_person_outfit ?name ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?goal)
	(printout t "prueba nuevo PLAN greet person")
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?color "_" ?outfit "_" ?ppl ",_tell_me_are_you_a_" ?color "_" ?outfit "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to_pace ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?color ?outfit ?place)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions set_plan_status ?name)(actions_num_params 3 8)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status question_1 greeted)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_follow_man
	?goal <- (objetive follow_man ?name ?ppl ?place ?step)
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

(defrule plan_follow_person
	?goal <- (objetive follow_person ?name ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name) (actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions find-object-man man no_location)(duration 6008)))
	(assert (plan (name ?name) (number 12) (actions set_plan_status ?name) (actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status man followed_person)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_follow_person_outfit
	?goal <- (objetive follow_person_outfit ?name ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?color "_" ?outfit "_" ?ppl ",_tell_me_are_you_a_" ?color "_" ?outfit "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?color ?outfit ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions find-object-man man no_location)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status man followed_person)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_guide_man
	?goal <- (objetive guide_man ?name ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person person ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name person finded)(actions_num_params 4 4) (duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object-man man_guide ?place2) (duration 6000)))
	(assert (plan (name ?name) (number 5)(actions set_plan_status ?name dummy)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_guide-person
	?goal <- (objetive guide_person ?name ?ppl ?peopleDsc ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?peopleDsc "_" ?ppl ",_tell_me_are_you_a_" ?peopleDsc "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions find-object-man man_guide ?place2)))
	(assert (plan (name ?name) (number 12)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_guide-person-outfit
	?goal <- (objetive guide_person_outfit ?name ?ppl ?color ?outfit ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(bind ?confirmation(str-cat "Hello_i_am_looking_for_a_" ?color "_" ?outfit "_" ?ppl ",_tell_me_are_you_a_" ?color "_" ?outfit "_" ?ppl))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-endurance-person person ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 8)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions find-endurance-person ?ppl ?color ?outfit ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status ?ppl find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name ?ppl find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 11)(duration 6000)))
	(assert (plan (name ?name) (number 11)(actions find-object-man man_guide ?place2)(duration 6000)))
	(assert (plan (name ?name) (number 12)(actions set_plan_status ?name)(actions_num_params 3 8)(duration 6000)))
	(assert (plan (name ?name) (number 13)(actions update_status man_guide followed_person)(duration 6000)))
	(assert (finish-planner ?name 13))
)

(defrule plan_describe-person-to-me
	?goal <- (objetive describe_person ?name ?posture ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
        ;(assert (plan (name ?name) (number 2)(actions gesture_person ?posture ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person person ?posture ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name person finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions scan_person ?posture ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions speech-person-description ?place) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions set_plan_status ?name dummy) (duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status question_2 ask)(duration 6000)))
	;(assert (plan (name ?name) (number 5)(actions answer_question question_2 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 9))	
)

(defrule plan_describe-person
	?goal <- (objetive describe_person ?name ?posture ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions gesture_person ?posture ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person person ?posture ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name person finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions scan_person ?posture ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions go_to_place ?place2)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person person ?place2)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions make_task ?name person finded)(actions_num_params 8 8)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions speech-person-description ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 9)(actions set_plan_status ?name dummy) (duration 6000)))
	(assert (plan (name ?name) (number 10)(actions update_status question_2 ask)(duration 6000)))
	;(assert (plan (name ?name) (number 6)(actions answer_question question_2 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 10))
)

;(defrule plan_remind-person
;	?goal <- (objetive remind_person ?name ?person ?place ?step)
;	=>
;	(retract ?goal)
;	(bind ?speech(str-cat "Hello, please look at me, I will try to remember you"))
;	(bind ?speech_1(str-cat "thank you very much, Now I can remember you"))
;	(assert (plan (name ?name) (number 1) (actions go_to_place ?place)(duration 6000)))
;	(assert (plan (name ?name) (number 2) (actions find-endurance-person person ?place)(duration 6000)))
;	(assert (plan (name ?name) (number 3) (actions speech-anything ?speech)(duration 6000)))
;	(assert (plan (name ?name) (number 4) (actions remind_person ?person ?place) (duration 6000)))
;	(assert (plan (name ?name) (number 5) (actions speech-anything ?speech_1) (duration 6000)))
;	(assert (plan (name ?name) (number 6) (actions set_plan_status ?name dummy) (duration 6000)))
;	(assert (plan (name ?name) (number 7) (actions update_status ?person reminded) (duration 6000)))
;	(assert (finish-planner ?name 7))
;)

(defrule plan_remind-person_v2
	?goal <- (objetive remind_person ?name ?person ?place ?step)
	=>
	(retract ?goal)
	(bind ?speech(str-cat "Hello " ?person ", please look at me, I will try to remember you"))
	(bind ?speech_1(str-cat "thank you very much, Now I can remember you"))
	(bind ?confirmation(str-cat "Hello, tell me are you " ?person ""))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3) (actions find-endurance-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task ?name person finded)(actions_num_params 5 13)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions confirmation ?confirmation) (duration 6000)))
	(assert (plan (name ?name) (number 6)(actions make_task ?name)(actions_num_params 7 8)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status person find)(duration 6000)))
	(assert (plan (name ?name) (number 8)(actions update_status conf true)(duration 6000)))
	(assert (plan (name ?name) (number 9)(actions repeat_task ?name person find)(actions_num_params 3 8 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 10)(actions make_task ?name)(actions_num_params 11 13)(duration 6000)))
	(assert (plan (name ?name) (number 11) (actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name ?name) (number 12) (actions remind_person ?person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 13) (actions speech-anything ?speech_1) (duration 6000)))
	(assert (plan (name ?name) (number 14) (actions set_plan_status ?name)(actions_num_params 3 8) (duration 6000)))
	(assert (plan (name ?name) (number 15) (actions update_status ?person reminded) (duration 6000)))
	(assert (finish-planner ?name 15))
)


(defrule plan_greet-known-person
	?goal <- (objetive greet_known_person ?name ?person ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 4 5) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions make_task ?name person finded)(actions_num_params 3 6)(duration 6000)))
	
	(assert (plan (name ?name) (number 3) (actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4) (actions find-endurance-person person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions find-reminded-person ?person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions repeat_task ?name ?person greet) (actions_num_params 4 5 3) (duration 6000)))
	;(assert (plan (name ?name) (number 5) (actions  )))
	(assert (plan (name ?name) (number 7) (actions set_plan_status ?name) (actions_num_params 4 5) (duration 6000)))
	(assert (plan (name ?name) (number 8) (actions update_status ?person greeted) (duration 6000)))
	(assert (finish-planner ?name 8))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; before split tasks

(defrule exe_find-how-many-people
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfhmp ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive find_how_many_people task_find_how_many_people ?ppl ?place ?step))
)

(defrule exe_find-how-many-people_dsc
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfhmp) (actor ?robot)(obj ?ppl)(from ?peopleDsc)(to ?place)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_how_many_people task_find_how_many_people ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_offer_eat_drink
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink task_offer_eat_drink ?ppl ?eatdrink ?place ?step))
)

(defrule exe_offer_eat_drink_dsc
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer ?ppl ?peopleDsc ?eatdrink ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink task_offer_eat_drink ?ppl ?peopleDsc ?eatdrink ?place ?step))
)

(defrule exe_bring-obj-to-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pbringtop ?ppl ?obj ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive bring_obj_to_prsn task_bring_obj_to_prsn ?ppl ?obj ?place ?step))
)

(defrule exe_bring-obj-to-prsn 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pbringtop ?ppl ?peopleDsc ?obj ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive bring_obj_to_prsn task_bring_obj_to_prsn ?ppl ?peopleDsc ?obj ?place ?step))
)

(defrule exe_bring-obj-to-prsn_color_outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pbringtop ?ppl ?color ?outfit ?obj ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive bring_obj_to_prsn task_bring_obj_to_prsn ?ppl ?color ?outfit ?obj ?place ?step))
)

(defrule exe_greet-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_person task_greet_person ?ppl ?place ?step))
)

(defrule exe_greet-person_dsc
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_person task_greet_person ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_greet-person-outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_outfit ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_person_outfit task_greet_person_outfit ?ppl ?color ?outfit ?place ?step))
)

(defrule exe_follow-man
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_man task_follow_man ?ppl ?place ?step))
)

(defrule exe_follow-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_person task_follow_person ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_follow-person-outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person_outfit ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_person_outfit task_follow_person_outfit ?ppl ?color ?outfit ?place ?step))
)

(defrule exe_guide-man
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_man task_guide_man ?ppl ?place1 ?place2 ?step))
)

(defrule exe_guide-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person ?ppl ?peopleDsc ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person task_guide_person ?ppl ?peopleDsc ?place1 ?place2 ?step))
)

(defrule exe_guide_person_outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person_outfit task_guide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step))
)

(defrule exe_describe-person-to-me
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pdescribe_person_to_me ?posture ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive describe_person task_describe_person ?posture ?place ?step))
)

(defrule exe_describe-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pdescribe_person ?posture ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive describe_person task_describe_person ?posture ?place1 ?place2 ?step))
)

(defrule exe_remind-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task premind_person ?person ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive remind_person task_remind_person ?person ?place ?step))
)

(defrule exe_greet-known-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_known_person ?place ?step)
	(item (name ?person) (status reminded))
	=>
	(retract ?f1)
	(assert (objetive greet_known_person task_greet_known_person ?person ?place ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
