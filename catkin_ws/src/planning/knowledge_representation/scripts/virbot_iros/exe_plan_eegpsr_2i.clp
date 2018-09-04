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


(defrule exe-plan-task-ask-for-incomplete-2
	?f <- (plan (name ?name) (number ?num-pln) (status active) (statusTwo active)(actions ask_for_incomplete ?item ?od) (actions_num_params ?ini ?end)(duration ?t))
	?f1 <- (item (name ?item))
	;?f2 <- (item (name incomplete))
	=>
	(bind ?command (str-cat "" ?item " " ?od "" ))
        (assert (send-blackboard ACT-PLN ask_inc ?command ?t 4))
	(modify ?f (statusTwo inactive))
	;(modify ?f2 (status nil))
)

(defrule exe-plan-ask-asked-for-incomplet-2
	?f <- (received ?sender command ask_inc ?item ?od $?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?item ?od) (actions_num_params ?ini ?end))
	=>
	(retract ?f)
	(assert (put_param $?param))
)

(defrule exe-plan-no-ask-for-incomplete-2
	?f <- (received ?sender command ask_inc ?item ?od $?param 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?item ?od) (actions_num_params ?ini ?end))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; get person description

(defrule exe-plan-get-person-description
	?f <- (plan (name ?name) (number ?num-pln) (status active) (statusTwo active)(actions get_person_description ?ppl ?place)(duration ?t))
	?f1 <- (item (name ?place))
	;?f2 <- (item (name incomplete))
	=>
	(bind ?command(str-cat "" ?ppl " " ?place ""))
	(assert (send-blackboard ACT-PLN get_person_description ?command ?t 4))
	(modify ?f (statusTwo inactive))
	;(modify ?f2 (status nil))
)

(defrule exe-plan-geted-person-description
	?f <- (received ?sender command get_person_description ?ppl ?place $?params 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_person_description ?ppl ?place))
	=>
	(retract ?f)
	(assert (put_param $?params ?place))
)

(defrule exe-plan-no-get-person-descrption 
	?f <- (received ?sender command get_person_description ?ppl ?place $?params 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (actions get_person_description ?ppl ?place))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; for set the incomplate place and for set the person description
(defrule exe-plan-task-set-param
	?f <- (plan (name ?name) (number ?num-pln) (status active) (statusTwo inactive)(actions_num_params ?ini ?end&:(neq ?ini ?end)))
	?f1 <- (put_param $?newparam)
	?f2 <- (plan (name ?name) (number ?ini) (actions $?params))
	=>
	(modify ?f (actions_num_params (+ 1 ?ini) ?end))
	(modify ?f2 (actions $?params $?newparam))
)

(defrule exe-plan-task-set-last-param
	?f <- (plan (name ?name) (number ?num-pln) (status active) (statusTwo inactive)(actions_num_params ?ini ?ini))
	?f1 <- (put_param $?newparam)
	?f2 <- (plan (name ?name) (number ?ini) (actions $?params))
	?f3 <- (item (name incomplete))
	=>
	(retract ?f1)
	(modify ?f (status accomplished)(statusTwo active))
	(modify ?f2 (actions $?params $?newparam))
	(modify ?f3 (status asked))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-find-endurance-person-ei 
	(plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person ?param1 ?param2 ?param3 ?param4 ?param5) (duration ?t))
	;?f1 <- (item (name ?ppl))
	=>
	(bind ?command (str-cat "" ?param1 " " ?param2 " " ?param3 " " ?param4 " " ?param5 ""))
	(assert (send-blackboard ACT-PLN find_e_person ?command ?t 4))
)

(defrule exe-plan-finded-endurance-person-ei 
	?f <- (received ?sender command find_e_person ?param1 ?param2 ?param3 ?param4 ?param5 1)
	?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person $?params))
	=>
	(retract ?f)
	(modify ?f1 (status finded))
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-finded-endurance-person-ei
	?f <- (received ?sender command find_e_person ?param1 ?param2 ?param3 ?param4 ?param5 0)
	;?f1 <- (item (name ?ppl))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-endurance-person $?params))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
