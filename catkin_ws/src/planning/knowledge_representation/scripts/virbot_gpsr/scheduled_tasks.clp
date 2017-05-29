;********************************************************
;*                                                      *
;*      exe_scheduled_tasks.clp                         *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*			Adrian Revueltas                *
;*                                                      *
;*                      1/20/14                         *
;*                                                      *
;********************************************************

(defrule exe_scheduled-if-conditional-true-location
        (item (name ?object) (zone ?status_object))
        (name-scheduled ?name ?ini ?end)
        ?f3<-(condition (conditional if) (arguments ?object zone ?status&:(eq ?status ?status_object) )
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?ts))(status active))
        ?f2 <- (state (name ?name) (number ?ts))
        =>
        (modify ?f1 (status inactive))
        (modify ?f2 (status active))
		(retract ?f3)
)




(defrule exe_scheduled-if-conditional-false-location
        (item (name ?object) (zone ?status_object))
        (name-scheduled ?name ?ini ?end)
        ?f3<-(condition (conditional if) (arguments ?object zone ?status&:(neq ?status ?status_object) )
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
        ?f2 <- (state (name ?name) (number ?fs))
        =>
        (modify ?f1 (status inactive))
        (modify ?f2 (status active))
		(retract ?f3)
)






(defrule exe_scheduled-if-conditional-true-status
        (item (name ?object) (status $?status_object))
        (name-scheduled ?name ?ini ?end)
        ;(condition (conditional if) (arguments ?object status $?status&:(eq $?status $?status_object)) 
        ?f3<-(condition (conditional if) (arguments ?object status $?status_object) 
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?ts))(status active))
	?f2 <- (state (name ?name) (number ?ts))
        =>
	(modify ?f1 (status inactive))
	(modify ?f2 (status active))
	(retract ?f3)
)



(defrule exe_scheduled-if-conditional-false-status
	(item (name ?object) (status $?status_object))
    	(name-scheduled ?name ?ini ?end)
	?f3 <- (condition (conditional if) (arguments ?object status $?status&:(neq $?status $?status_object) )
		(true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
	?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
	?f2 <- (state (name ?name) (number ?fs))
	=>
	(modify ?f1 (status inactive))
	(modify ?f2 (status active))
	(retract ?f3)
)




(defrule exe_scheduled-attend-object
	(state (name ?name) (number ?num-state)(status active)(duration ?time))
        (name-scheduled ?name ?ini ?end)
	(item (name ?object)(shared true))
        (cd-task (cd attend) (actor ?actor)(obj ?object)(from ?device)(to status)(name-scheduled ?name)(state-number ?num-state) )
        =>
	(bind ?command (str-cat "" ?device " " ?object""))
	(assert (send-blackboard ACT-PLN status_object ?command ?time 4))

)


(defrule exe_scheduled-speak
	(state (name ?name) (number ?num-state)(status active))
    	(name-scheduled ?name ?ini ?end)
	?f1 <- (cd-task (cd speak) (actor robot)(message ?message)(name-scheduled ?name)(state-number ?num-state))
        =>
	(retract ?f1)
        (bind ?command (str-cat "" ?message""))
        (assert (send-blackboard ACT-PLN speak ?command 0 0))
)


(defrule exe_scheduled-ptrans
        (state (name ?name) (number ?num-state)(status active)(duration ?time))
	?f2 <- (item (name ?robot)(zone ?zone));;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (item (name ?to)(pose ?x ?y ?z));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,jc
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd ptrans) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
        =>
        (retract ?f1);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (bind ?command (str-cat "" ?robot " " ?to " " ?x " " ?y " " ?z""))
        (assert (send-blackboard ACT-PLN goto ?command ?time 4))
		(modify ?f2 (zone frontexit));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
)


(defrule exe_scheduled-ptrans-objects
        (state (name ?name) (number ?num-state)(status active)(duration ?time))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd ptrans) (actor robot)(obj ?obj)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	(item (type Objects) (name ?obj))
	(item (type Objects) (name ?to))
        =>
        (retract ?f1)
	(assert (goal (move ?obj)(on-top-of ?to)) )
)




