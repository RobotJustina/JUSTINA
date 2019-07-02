;;;;;;;;;;;;;
(defrule task_get_object_without_place
	?f <- (task ?plan get_object ?param1 ?step)
	?f1 <- (item (type Objects)(name ?param1))
	=>
	(retract ?f)
	(assert (task ?plan get_object ?param1 default_location ?step))
)
;;;;;;;;;;;;;
