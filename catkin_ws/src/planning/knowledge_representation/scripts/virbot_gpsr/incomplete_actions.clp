

;ask_info (params object)
;aks_info (params place)

;;;;;;;;; TASKS incompletes
; find_person_in_room -Person- ?Location
; get_object ?Object default_location
; handover_object ?Object

(defrule task_ask_info
	?f <- (task ?plan ask_info ?param ?step)
	?f1 <- (item (name ?param)) 
	=>
	(retract ?f)
	(printout t "Ask for INFO" crlf)
	(assert (condition (conditional if) (arguments ?param status asked)(true-state (+ ?step 1))(false-state ?step)(name-schedule ?plan)(state-number ?step)))
	(assert (cd-task (cd pask_info)(actor robot) (obj robot)(from ?plan)(to ?param) (name-schedule ?plan)(state-number ?step)))
	(modify ?f1 (status nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;; SPLIT IN SUBTAREAS

(defrule plan_ask_info
	?goal <- (objetive ask_info ?name ?param ?plan ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo plan ask for info" crlf)
	(assert (plan (name ?name) (number 1) (actions ask_info ?param)(duration 6000)))
	(assert (finish-planner ?name 1))
)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RULES BEFORE SPLIT


(defrule exe_ask_info
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-schedule ?name ?ini ?end)
	?f1 <- (cd-task (cd pask_info) (actor ?robot) (obj ?robot) (from ?plan) (to ?param) (name-schedule ?name) (state-number ?step))
	=>
	(retract ?f1)
	(assert (objetive ask_info task_ask_info ?param ?plan ?step))
)






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,
;;;;;;;;;;;;;;; EXE PLAN

(defrule exe_plan_ask_for_info
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_info ?param ?plan) (duration ?t))
	?f1 <- (item (name ?param) (status ?x&:(neq ?x asked)))
	=>
	(bind ?command (str-cat "" ?param " " ?plan ""))
	(assert (send-blackboard ACT-PLN ask_info ?command ?t 4))
)

(defrule exe_plan_ask_for_info_af_place
	?f <- (received ?sender command ask_info ?param ?plan ?place 1)
	?f1 <- (item (name ?place)(type ?x&:(or (eq ?x Room) (eq ?x Forniture))))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_info ?param ?plan))
	?f3 <- (task ?plan find_person_in_room ?person ?step)
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
	(retract ?f3)
	(assert (task ?plan find_person_in_room ?person ?place ?step))
)


(defrule exe_plan_ask_for_info_af_obj
	?f <- (received ?sender command ask_info ?param ?plan ?object 1)
	?f1 <- (item (name ?object)(type Object))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_info ?param ?plan))
	?f3 <- (task ?plan get_object default_location ?step1)
	?f4 <- (task ?plan handover_object ?step2)
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
	(retract ?f3)
	(retract ?f4)
	(assert (task ?plan get_object ?object default_location ?step1))
	(assert (task ?plan handover_object ?object ?step2))
)

(defrule exe_plan_ask_for_info_neg
	?f <- (received ?sender command ask_info ?param ?plan ?info 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_info ?param ?plan))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)


