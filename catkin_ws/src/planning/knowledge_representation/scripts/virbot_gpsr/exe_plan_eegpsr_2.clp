;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	23/04/2018			;;;
;;;					;;;
;;;	Julio Cesar Cruz Estrada 	;;;
;;;					;;;
;;;	EEGPSR				;;;
;;;	category II			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;find many people task
(defrule exe-plan-find-many-people-dsc 
	(plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?place "" ))
        (assert (send-blackboard ACT-PLN find_many_people ?command ?t 4))
)

(defrule exe-plan-finded-many-people-dsc 
	?f <- (received ?sender command find_many_people ?ppl ?peopleDsc ?place ?speech 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place))
	?f3 <- (item (name speech))
	?f4 <- (item (name ?peopleDsc) (image ?sp2))
	=>
	(retract ?f)
	;(modify ?f1 (status finded))
	(bind ?cm(str-cat "" ?speech "_" ?sp2 "_" ?peopleDsc ""))
	(modify ?f2 (status accomplished))
	(modify ?f3 (image ?cm))
)

(defrule exe-plan-no-finded-many-people-dsc 
	?f <- (received ?sender command find_many_people ?ppl ?peopleDsc ?place ?speech 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place))
	?f3 <- (item (name speech))
	?f4 <- (item (name ?peopleDsc) (image ?sp2))
	=>
	(retract ?f)
	(bind ?cm (str-cat "" ?speech "_" ?sp2 "_" ?peopleDsc ""))
	(modify ?f2 (status active))
	(modify ?f3 (image ?cm))
)

(defrule exe-plan-find-many-people  
	(plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?place "" ))
        (assert (send-blackboard ACT-PLN find_many_people ?command ?t 4))
)

(defrule exe-plan-finded-many-people
	?f <- (received ?sender command find_many_people ?ppl ?place ?speech 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?place))
	?f3 <- (item (name speech))
	=>
	(retract ?f)
	;(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
	(modify ?f3 (image ?speech))
)

(defrule exe-plan-no-finded-many-people
	?f <- (received ?sender command find_many_people ?ppl ?place ?speech 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?place))
	?f3 <- (item (name speech))
	=>
	(retract ?f)
	(modify ?f2 (status active))
	(modify ?f3 (image ?speech))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; get amount of people

(defrule exe-plan-get-amount-people
	(plan (name ?name) (number ?num-pln) (status active) (actions get_amount_people ?place) (duration ?t))
	;?f1 <- (item (name offer))
	=>
	(bind ?command (str-cat "" ?place ""))
	(assert (send-blackboard ACT-PLN amount_people ?command ?t 4))
)

(defrule exe-plan-geted-amount-people
	?f <- (received ?sender command amount_people ?place 1)
	;?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_amount_people ?place))
	=>
	(retract ?f)
	;(modify ?f1 (status accomplished))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-get-amount-people
	?f <- (received ?sender command amount_people ?place 0)
	;?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_amount_people ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;; 
;;; ask and offer drink o something to eat
(defrule exe-plan-ask-and-offer
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink ?place) (duration ?t))
	?f1 <- (item (name offer))
	=>
	(bind ?command (str-cat "" ?ppl " " ?eatdrink " " ?place ""))
	(assert (send-blackboard ACT-PLN ask_and_offer ?command ?t 4))
)

(defrule exe-plan-asked-and-offered
	?f <- (received ?sender command ask_and_offer ?ppl ?eatdrink ?place 1)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink ?place))
	=>
	(retract ?f)
	;(modify ?f1 (status final_offer))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-asked-nor-offered
	?f <- (received ?sender command ask_and_offer ?ppl ?eatdrink ?place 0)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-ask-and-offer-dsc
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place)(duration ?t))
	?f1 <- (item (name offer))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?eatdrink " " ?place ""))
	(assert (send-blackboard ACT-PLN ask_and_offer ?command ?t 4))
)

(defrule exe-plan-asked-and-offered-dsc
	?f <- (received ?sender command ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place 1)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place))
	=>
	(retract ?f)
	;(modify ?f1 (status final_offer))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-asked-nor-offer-dsc
	?f <- (received ?sender command ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place 0)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; find person endurance task 

(defrule exe-plan-find-endurance-person
	(plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command(str-cat "" ?ppl " " ?place ""))
	(assert (send-blackboard ACT-PLN find_e_person ?command ?t 4))
)

(defrule exe-plan-finded-endurance-person
	?f <- (received ?sender command find_e_person ?ppl ?place 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?place))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-pan-no-finded-endurance-person
	?f <- (received ?sender command find_e_person ?ppl ?place 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-find-endurance-person-dsc 
	(plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?place ""))
	(assert (send-blackboard ACT-PLN find_e_person ?command ?t 4))
)

(defrule exe-plan-finded-endurance-person-dsc 
	?f <- (received ?sender command find_e_person ?ppl ?peopleDsc ?place 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-finded-endurance-person-dsc 
	?f <- (received ?sender command find_e_person ?ppl ?peopleDsc ?place 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-find-endurance-person-outfit
	(plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?color ?outfit ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?color " " ?outfit " " ?place ""))
	(assert (send-blackboard ACT-PLN find_e_person ?command ?t 4))
)

(defrule exe-plan-finded-endurance-person-outfit
	?f <- (received ?sender command find_e_person ?ppl ?color ?outfit ?place 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?color ?outfit ?place))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-finded-endurance-person-outfit
	?f <- (received ?sender command find_e_person ?ppl ?color ?outfit ?place 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?color ?otfit ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;
;;; scan person task
(defrule exe-plan-scan-person
	(plan (name ?name) (number ?num-pln) (status active) (actions scan_person ?posture ?place) (duration ?t))
	?f1 <- (item (name ?place))
	=>
	(bind ?command (str-cat "" ?posture " " ?place ""))
	(assert (send-blackboard ACT-PLN scan_person ?command ?t 4))
)

(defrule exe-plan-scanned-person
	?f <- (received ?sender command scan_person ?posture ?place 1)
	?f1 <- (item (name ?place))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions scan_person ?posture ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-scan-person
	?f <- (received ?sender command scan_person ?posture ?place 0)
	?f1 <- (item (name ?place))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions scan_person ?posture ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;
;;; remind person
(defrule exe-plan-remind-person
	(plan (name ?name) (number ?num-pln) (status active) (actions remind_person ?person ?place) (duration ?t))
	?f1 <- (item (name ?person))
	=>
	(bind ?command (str-cat "" ?person " " ?place ""))
	(assert (send-blackboard ACT-PLN remind_person ?command ?t 4))
)

(defrule exe-plan-reminded-person
	?f <- (received ?sender command remind_person ?person ?place 1)
	?f1 <- (item (name ?person))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions remind_person ?person ?place))
	=>
	(retract ?f)
	;(modify ?f1 (status reminded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-reminded-person
	?f <- (received ?sender command remind_person ?person ?place 0)
	?f1 <- (item (name ?person))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions remind_person ?person ?place))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; poner un status a los planes
(defrule exe-plan-set-plan-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name) (actions_num_params ?ini ?end) (statusTwo active))
	?f1 <- (finish-planner ?name ?n)
	=>
	(retract ?f1)
	(assert (set_plan_status ?name ?n))
	(modify ?f (statusTwo inactive))
)

(defrule exe-plan-seted-status-plan
	(set_plan_status ?name ?n)
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name) (actions_num_params ?ini ?end&:(< ?ini ?end)) (statusTwo inactive))
	?f1 <- (plan (name ?name) (number ?ini)(status inactive)(statusTwo ?st&:(neq ?st plan_active)))
	=>
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
	(modify ?f1 (statusTwo plan_active))
)

(defrule exe-plan-last-stated-status-plan
	?f2 <-(set_plan_status ?name ?n)
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name) (actions_num_params ?ini ?ini) (statusTwo inactive))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive) (statusTwo ?st&:(neq ?st plan_active)))
	=>
	(retract ?f2)
	(modify ?f (status accomplished))
	(modify ?f1 (statusTwo plan_active))
	(assert (finish-planner ?name ?n))
)

(defrule exe-plan-reset-status-plan
	(set_plan_status ?name ?n)
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name) (actions_num_params ?ini ?end&:(< ?ini ?end)) (statusTwo inactive))
	?f1 <- (plan (name ?name) (number ?ini) (statusTwo plan_active))
	=>
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
	(modify ?f1 (status accomplished) (statusTwo active))
)

(defrule exe-plan-reset-last-plan
	?f2 <- (set_plan_status ?name ?n)
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name) (actions_num_params ?end ?end) (statusTwo inactive))
	?f1 <- (plan (name ?name) (number ?end) (statusTwo plan_active))
	=>
	(retract ?f2)
	(modify ?f (status accomplished))
	(modify ?f1 (status accomplished)(statusTwo active))
	(assert (finish-planner ?name ?n))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; find reminded person

(defrule exe-plan-find-reminded-person
	(plan (name ?name) (number ?num-pln) (status active) (actions find-reminded-person ?person ?place) (duration ?t))
	?f <- (item (name ?person))
	=>
	(bind ?command (str-cat "" ?person " " ?place ""))
	(assert (send-blackboard ACT-PLN find_reminded_person ?command ?t 4))
)

(defrule exe-plan-finded-reminded-person
	?f <- (received ?sender command find_reminded_person ?person ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-reminded-person ?person ?place)) 
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(modify ?f2 (status greet))
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-no-find-reminded-person
	?f <- (received ?sender command find_reminded_person ?person ?place 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-reminded-person ?person ?place))
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,
;;;;;;; repeat the tasks
(defrule exe-plan-no-repeat-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions repeat_task ?name ?item ?status) (actions_num_params ?ini ?end ?ini_rep ?end_rep))
	(item (name ?item) (status ?status))
	=>
	(modify ?f (status accomplished))
)

(defrule exe-plan-repeat-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions repeat_task ?name ?item ?status) (actions_num_params ?ini ?end ?ini_rep ?end_rep&:(< ?ini_rep ?end_rep)))
	(item (name ?item) (status ?st&:(neq ?st ?status)))
	?f1 <- (finish-planner ?name ?n)
	=>
	(retract ?f1)
	(assert (enable_repetition ?name ?n ?ini ?end))
	(modify ?f (actions_num_params ?ini ?end (+ ?ini_rep 1) ?end_rep))
)

(defrule exe-plan-repeat-last-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions repeat_task ?name ?item ?status) (actions_num_params ?ini ?end ?end_rep ?end_rep))
	(item (name ?item) (status ?st&:(neq ?st ?status)))
	(finish-planner ?name ?n)
	=>
	(modify ?f (status accomplished))
)

(defrule exe-plan-enable-repetition
	?f <- (plan (name ?name)(number ?num-pln)(status active)(actions repeat_task ?name ?item ?status) (actions_num_params ?ini ?end&:(< ?ini ?end) ?ini_rep ?end_rep))
	?f2 <- (plan (name ?name2) (number ?ini)(statusTwo plan_active))
	(enable_repetition ?name ?n ?ini2 ?end2)
	=>
	(modify ?f (actions_num_params (+ ?ini 1) ?end ?ini_rep ?end_rep))
	(modify ?f2 (status inactive))
)

(defrule exe-plan-enable-last-repetition
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions repeat_task ?name ?item ?status) (actions_num_params ?end ?end ?ini_rep ?end_rep))
	?f2 <- (plan (name ?name2) (number ?end) (statusTwo plan_active))
	?f3 <- (enable_repetition ?name ?n ?ini2 ?end2)
	=>
	(retract ?f3)
	(assert (finish-planner ?name ?n))
	(modify ?f  (status inactive)(actions_num_params ?ini2 ?end2 ?ini_rep ?end_rep))
	(modify ?f2 (status inactive))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; speech response

(defrule exe-plan-speech-response 
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-response ?sp) (duration ?t))
	?f <- (item (name ?sp) (image ?speech))
	=>
	(bind ?command (str-cat "" ?speech ""))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speeched-response
	?f <- (received ?sender command spg_say ?speech 1)
	?f1 <-(plan (name ?name) (number ?num-pln) (status active) (actions speech-response ?sp))
	;?f2 <- (item (name ?sp) (image ?speech))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-no-speech-response
	?f <- (received ?sender command spg_say ?speech 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech-response ?sp))
	;?f2 <- (item (name ?sp) (image ?speech))
	=>
	(retract ?f)
	(modify ?f1 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;; move actuator eegpsr

(defrule exe-plan-move-actuator-eegpsr
        (plan (name ?name) (number ?num-pln)(status active)(actions move_eegpsr ?actuator ?obj)(duration ?t))
 	    (item (name ?obj) (pose ?x ?y ?z) )
        (Arm (name ?arm) (status ready)(bandera ?id) (grasp ?obj))
        =>
        (bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
)



(defrule exe-plan-moved-actuator-eegpsr
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move_eegpsr ?actuator ?object))
	?f3 <- (item (name robot));;;;;;;;;; T1 test for quit grasp object subtask
	;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f3 (hands ?object));;;;; T1 test
	(modify ?f1 (status grabed));;;;;; T1 test
        ;(retract ?f3)
)


(defrule exe-plan-no-moved-actuator-eegpsr
	?f <- (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
	?f1 <- (item (name ?object))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions move_eegpsr ?actuator ?object))
	?f3 <- (Arm (name ?arm) (status ready) (bandera ?id) (grasp ?object))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status nil) (grasp nil))
)
;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; speech order

(defrule exe-plan-speech-order
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-order)(duration ?t))
	(order ?sp)
	(num_order ?num&:(neq ?num 1))
	=>
	(bind ?command(str-cat "Human_please_help_me_to_serve_custumers_" ?sp ))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speech-zero-order
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-order)(duration ?t))
	(order ?sp)
	(num_order 1)
	=>
	(bind ?command(str-cat "Sorry_i_can_not_take_any_order"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speeched-order
	?f <- (received ?sender command spg_say $?spc 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech-order))
	?f2 <- (order ?sp)
	?f3 <- (num_order ?num)
	=>
	(retract ?f ?f2 ?f3)
	(modify ?f1 (status accomplished))
	(assert (order _))
	(assert (num_order 1))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;; speech person description
(defrule exe-plan-speech-person-description
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-person-description ?place)(duration ?t))
	(person_description ?sp&:(neq ?sp _))
	=>
	(bind ?command(str-cat "Hello_I_am_going_to_describe_the_person_i_met_in_the_" ?place ",_" ?sp ))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speech-no-person-description 
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-person-descrption ?place)(duration ?t))
	(person_description _)
	=>
	(bind ?command(str-cat "Sorry_i_can_not_get_the_description_of_the_person_at_" ?place))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speeched-person-description 
	?f <- (received ?sender command spg_say $?spc 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech-person-description ?place))
	?f2 <- (person_description ?sp)
	=>
	(retract ?f ?f2)
	(modify ?f1 (status accomplished))
	(assert (person_description _))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-ask-and-offer_no_place
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink) (duration ?t))
	?f1 <- (item (name offer))
	=>
	(bind ?command (str-cat "" ?ppl " " ?eatdrink ""))
	(assert (send-blackboard ACT-PLN ask_and_offer ?command ?t 4))
)

(defrule exe-plan-asked-and-offered_no_place
	?f <- (received ?sender command ask_and_offer ?ppl ?eatdrink 1)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink))
	=>
	(retract ?f)
	;(modify ?f1 (status final_offer))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-asked-nor-offered_no_place
	?f <- (received ?sender command ask_and_offer ?ppl ?eatdrink 0)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?eatdrink))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
