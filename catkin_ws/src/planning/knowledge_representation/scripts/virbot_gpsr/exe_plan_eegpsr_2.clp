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
(defrule exe-plan-find-many-people 
	(plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?place "" ))
        (assert (send-blackboard ACT-PLN find-many-people ?command ?t 4))
)

(defrule exe-plan-finded-many-people
	?f <- (received ?sender command find-many-people ?ppl ?peopleDsc ?place 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-finded-many-people
	?f <- (received ?sender command find-many-people ?ppl ?peopleDsc ?place 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-many-people ?ppl ?peopleDsc ?place))
	=>
	(retract ?f)
	(modify ?f2 (status active))

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
	?f <- (received ?sender command amount_people ?command ?t 4)
	;?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_amount_people ?place))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;; 
;;; ask and offer drink o something to eat
(defrule exe-plan-ask-and-offer
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place)(duration ?t))
	?f1 <- (item (name offer))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?eatdrink " " ?place ""))
	(assert (send-blackboard ACT-PLN ask_and_offer ?command ?t 4))
)

(defrule exe-plan-asked-and-offered
	?f <- (received ?sender command ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place 1)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place))
	=>
	(retract ?f)
	(modify ?f1 (status final_offer))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-asked-nor-offer 
	?f <- (received ?sender command ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place 0)
	?f1 <- (item (name offer))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; find person endurance task 
(defrule exe-plan-find-endurance-person
	(plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place) (duration ?t))
	?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?ppl " " ?peopleDsc " " ?place ""))
	(assert (send-blackboard ACT-PLN find_e_person ?command ?t 4))
)

(defrule exe-plan-finded-endurance-person
	?f <- (received ?sender command find_e_person ?ppl ?peopleDsc ?place 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-finded-endurance-person
	?f <- (received ?sender command find_e_person ?ppl ?peopleDsc ?place 0)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?ppl ?peopleDsc ?place))
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
;;;