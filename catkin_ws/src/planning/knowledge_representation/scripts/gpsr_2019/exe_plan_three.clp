;********************************************************
;*                                                      *
;*      actions_three.clp                               *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      03/04/2019                      *
;*                                                      *
;********************************************************


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  get bag task
(defrule exe-plan-task-get-bag 
	(plan (name ?name) (number ?num-pln) (status active) (actions get_bag) (duration ?t))
	=>
	(bind ?command (str-cat "get_bag"))
	(assert (send-blackboard ACT-PLN cmd_get_bag ?command ?t 4))
)

(defrule exe-plan-task-geted-bag 
	?f <- (received ?sender command cmd_get_bag get_bag 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
        ?f2 <- (Arm (name right))
	?f3 <- (item (name bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status ready) (grasp bag))
	(modify ?f3 (status grabed))
)

(defrule exe-plan-task-no-geted-bag 
	?f <- (received ?sender command cmd_get_bag get_bag 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;; follow to the taxi, uber, cab
(defrule exe-plan-task-follow-task
	(plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place) (duration ?t))
	=>
	(bind ?command (str-cat "" ?place ""))
	(assert (send-blackboard ACT-PLN cmd_follow_to_taxi ?command ?t 4))
)

(defrule exe-plan-task-followed-to-taxi 
	?f <- (received ?sender command cmd_follow_to_taxi ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	?f2 <- (item (name man))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status followed))
)

(defrule exe-plan-no-followed-to taxi
	?f <- (received ?sender command cmd_follow_to_taxi ?place 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
