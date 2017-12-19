;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;	18/12/2017			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-review-stack
	(plan (name ?name) (number ?num-pln) (status active) (actions review)(duration ?t))
	=>
	(bind ?command (str-cat "review"))
        (assert (send-blackboard ACT-PLN cmd_rstack ?command ?t 4))
)

(defrule exe-plan-reviewed-stack
        ?f <-  (received ?sender command cmd_rstack ?object 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions review))
	?f3 <- (item (name stack))
	=>
	(retract ?f)
	;(modify ?f3 (status review))
	
)

(defrule exe-plan-no-reviewed-stack
	?f <- (received ?sender command cmd_rstack ?object 0)
	?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions review))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

(defrule exe-plan-compare-stack
	(plan (name ?name) (number ?num-pln) (status active) (actions simul_plan)(duration ?t))
	=>
	(bind ?command (str-cat "compare"))
	(assert (send-clips ACT-PLN cmd_cstack ?command ?t 4))
)

(defrule exe-plan-compared-stack
	?f <- (received ?sender command cmd_cstack ?object 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions compare))
	?f3 <- (item (name stack))
	=>
	(retract ?f)
	(modify ?f3 (status review))
)

(defrule exe-plan-no-compared-stack
	?f <- (received ?sender command cmd_cstack ?object 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions compare))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; reglas para comprobar si las pilas cambiaron

(defrule transform_second_to_compare
	?f2 <- (stack_second $?pile)
	=>
	(assert (stack_compare $?pile))
)

(defrule compare_stacks_first_comparition
	?f2 <- (stack_compare $?pile2)
	(stack $?pile&:(eq $?pile $?pile2))
	?f <- (item (name stack) (status nil))
	=>
	(modify ?f (status first_comparition))
	(retract ?f2)
	
)

(defrule compare_stacks_second_comparition
	?f2 <- (stack_compare $?pile2)
	(stack $?pile&:(eq $?pile $?pile2))
	?f <- (item (name stack) (status first_comparition))
	=>
	(modify ?f (status review))
	(retract ?f2)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; reglas para hacer el plan simulado




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
