;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;	9/03/2018			;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;; save the initial cubes configuration
(defrule set_stacks
	?f <- (stack_dynamic $?pile)
	=>
	(retract ?f)
	(assert (stack $?pile))
	(assert (stack_back $?pile))
)

;;;;;;;;;;;; save the modified cubes configuration
(defrule set_modified_stacks
	?f <- (stack_review $?pile)
	=>
	(retract ?f)
	(assert (stack_modified $?pile))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; reglas para restaurar las pilas originales
(defrule delate_actual_stacks
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks_dynamic delate))
	?f1 <- (stack $?pile)
	=>
	(retract ?f1)
	(modify ?f (status active))
)

(defrule validate_no_stacks
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks_dynamic delate))
	(not (stack $?pile))
	=>
	(modify ?f (actions restore_stacks_dynamic restore))
)

(defrule restore_stack_dynamic
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks_dynamic restore))
	?f1 <- (stack_back $?pile)
	=>
	(retract ?f1)
	(modify ?f (status active))
	(assert (stack $?pile))
	(assert (stack_bkt $?pile))
)

(defrule validate_all_stacks_restore
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks_dynamic restore))
	(not (stack_back $?pile))
	=>
	(modify ?f (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; reglas para hacer el backtrackin con mas de dos pilas

(defrule compare-dynamic-stacks
	?f <- (stack_modified $?pile)
	?f1 <- (stack_bkt $?pile)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f ?f1)
	(modify ?f2 (status active))
)

(defrule compare-dynamic-stacks-no-match
	?f <- (stack_modified $?pile)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	(not (stack_bkt $?pile1&:(eq $?pile $?pile1) )
	=>
	(retract ?f)
	(modify ?f1 (status active))
)

(defrule validate_finish_comparition
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	(not (stack_modified $?pile))
	=>
	(assert (finish comparition))
)

(defrule exe-plan-speech-dynamic-stacks-no-change
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	?f <- (finish comparition)
	(not (stack_bkt $?pile))
	=>	
	(retract ?f)
	(bind ?command (str-cat "Cubes configuration did not change"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speech-dynamic-stacks-change
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))	
	?f <- (finis comparition)
	(stack_bkt $?pile)
	=>
	(retract ?f)
	(bind ?command (str-cat "I realize cubes configuration is different, I will explain what I think happened"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speeched-dynamic-stack
	?f <- (received ?sender command spg_say ?speech 1)
	?f2 <- (plan (name ?name) (number ?num-pln)(status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-speeched-dynamic-stack
	?f <- (received ?sender command spg_say ?speech 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

