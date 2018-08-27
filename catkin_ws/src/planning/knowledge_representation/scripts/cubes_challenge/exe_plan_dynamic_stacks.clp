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
	;(assert (stack_back $?pile))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule make-stack-backup
	?f <- (stack $?pile)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions backup_stacks first))
	=>
	(retract ?f)
	(assert (stack_dyn $?pile))
	(modify ?f1 (status active))
)

(defrule make-stack-backup-second 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions backup_stacks first))
	(not (stack $?pile))
	=>
	(modify ?f (actions backup_stacks second))
)

(defrule make-backup-third
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions backup_stacks second))
	?f1 <- (stack_dyn $?pile)
	=>
	(retract ?f1)
	(assert (stack $?pile))
	(assert (stack_back $?pile))
)

(defrule make-backup-forth
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions backup_stacks second))
	(not (stack_dyn $?pile))
	=>
	(modify ?f (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
	(not (stack_bkt $?pile1&:(eq $?pile $?pile1)))
	=>
	(retract ?f)
	(modify ?f1 (status active))
	(assert (stack_make_bkt $?pile))
)

(defrule validate_finish_comparition
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	(not (stack_modified $?pile))
	=>
	(assert (finish comparition))
)

(defrule exe-plan-speech-dynamic-stacks-no-change
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state)(duration ?t))
	?f <- (finish comparition)
	(not (stack_bkt $?pile))
	?f1 <- (simul_moves ?num)
	?f2 <- (item (name speech_1))
	=>	
	(retract ?f ?f1)
	(bind ?command (str-cat "Cubes configuration did not change"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	(assert (simul_moves 0))
	(assert (stack no_change))
	(modify ?f2 (image the_original_plan_will_not_change,_I_start_to_execute_it))
)

(defrule exe-plan-speech-dynamic-stacks-change
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state) (duration ?t))	
	?f <- (finish comparition)
	(stack_bkt $?pile)
	?f1 <- (simul_moves ?num)
	?f2 <- (item (name speech_1))
	=>
	(retract ?f ?f1)
	(bind ?command (str-cat "I realize cubes configuration is different, I will explain what I think happened"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	(assert (simul_moves 0))
	(modify ?f2 (image that_is_what_happened,_Now_I_make_a_new_plan_for_complete_the_command))
)

(defrule exe-plan-speeched-dynamic-stack
	?f <- (received ?sender command spg_say ?speech 1)
	?f2 <- (plan (name ?name) (number ?num-pln)(status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f)
	;(modify ?f2 (status accomplished))
	(assert (end speech_dynamic_stack_state))
	(assert (delate stack_bkt))
)

(defrule exe-plan-no-speeched-dynamic-stack
	?f <- (received ?sender command spg_say ?speech 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

(defrule delate-stack-bkt
	?f <- (delate stack_bkt)
	?f1 <- (stack_bkt $?pile)
	=>
	(retract ?f ?f1)
	(assert (delate stack_bkt))
)

(defrule validate-no-stack-bkt
	?f <- (delate stack_bkt)
	?f1 <- (end speech_dynamic_stack_state)
	(not (stack_bkt $?pile))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_dynamic_stack_state))
	=>
	(retract ?f ?f1)
	(modify ?f2 (status accomplished))
)

(defrule delate-modified-stack-empty
	;?f <- (stack_modified)
	?f1 <- (stack_make_bkt)
	=>
	(retract ?f1)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;; put the stacks base

(defrule exe-plan-put-base-block
	?f <- (stack_make_bkt $?rest ?base)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	?f2 <- (simul_moves ?num)
	=>
	(retract ?f ?f2)
	(modify ?f1 (status active))
	(assert (move ?base cubestable (+ ?num 1)))
	(assert (stack_set_base $?rest ?base))
	(assert (simul_moves (+ ?num 1)))
)

(defrule activate-stack-to-drain
	(not (stack_make_bkt $?pile))
	(stack_set_base $?rest ?base)
	(plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking pop))
	(not (drain ?someblock))
	=>
	(assert (drain ?base))
)

(defrule activate-stack-set-base-revision-for-delate
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	(not (stack_make_bkt $?pile))
	(stack_set_base $?rest ?base)
	(not (drain ?someblock))
	(not (delate stack_base))
	=>
	(assert (drain ?base))
	(assert (delate stack_base))
	(modify ?f (actions make-backtracking pop))
)
	


(defrule exe-plan-pop-stacks-base
	(plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking pop))
	(not (stack_make_bkt $?pile))
	?f2 <- (stack_set_base $?rest ?block&:(neq ?block nil) ?base)
	(move ?base cubestable ?num)
	?f <- (drain ?base)
	?f1 <- (simul_moves ?number)
	=>
	(retract ?f ?f1 ?f2)
	(assert (drain ?block))
	(assert (move ?block ?base (+ ?number 1)))
	(assert (simul_moves (+ ?number 1)))
	(assert (stack_set_base $?rest ?block))
)

(defrule exe-plan-pop-stacks-block
	(plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking pop))
	(not (stack_make_bkt $?pile))
	?f <- (stack_set_base $?rest ?block1&:(neq ?block1 nil) ?block2)
	(move ?block2 ?block&:(neq ?block cubestable) ?num)
	?f1 <- (drain ?block2)
	?f2 <- (simul_moves ?number)
	=>
	(retract ?f ?f1 ?f2)
	(assert (drain ?block1))
	(assert (move ?block1 ?block2 (+ ?number 1)))
	(assert (simul_moves (+ ?number 1)))
	(assert (stack_set_base $?rest ?block1))
)

(defrule delate_stack_base
	?f <- (stack_set_base ?block)
	?f1 <- (drain ?block)
	=>
	(retract ?f ?f1)
)

(defrule validate-no-stack-set-base
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking pop))
	(not (stack_set_base $?pile))
	?f1 <- (delate stack_base)
	=>
	(retract ?f1)
	(assert (start simul))
	(modify ?f (actions make-backtracking))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
