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
	(modify ?f2 (status accomplished))
	
)

(defrule exe-plan-no-reviewed-stack
	?f <- (received ?sender command cmd_rstack ?object 0)
	?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions review))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-compared-stack
	?f <- (received ?sender command cmd_cstack ?object 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions compare))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-speech-stack-no-change
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_stack_state) (duration ?t))
	(item (name stack) (status equal))
	?f <- (item (name speech_1)(status nil))
	=>
	(bind ?command (str-cat "The stacks do not change"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	(modify ?f (status no-change-stack)(image i_start_to_execute_the_command))
 )

(defrule exe-plan-speech-stack-change-first-stack
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_stack_state) (duration ?t))
	(item (name stack) (status first_comparition))
	(pile (name simul) (first_stack $?pile))
	?f <- (stack_compare $?pile)
	;?f1 <- (item (name speech_1))
	=>
	(retract ?f)
	(bind ?command (str-cat "I realize one stack is diferent, I will begin to explain what happened"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	;(modify ?f1 (image I_finish_the_simulation_I_start_to_execute_the_command))
)

(defrule exe-plan-speech-stack-change-second-stack
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_stack_state) (duration ?t))
	(item (name stack) (status first_comparition))
	(pile (name simul) (second_stack $?pile))
	?f <- (stack_compare $?pile)
	;?f1 <- (item (name speech_1))
	=>
	(retract ?f)
	(bind ?command (str-cat "I realize one stack is diferent, I will begin to explein what happened"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	;(modify ?f1 (image I_finish_the_simulation_I_start_to_execute_the_command))
)

(defrule exe-plan-speech-stack-change-two
	(plan (name ?name) (number ?num-pln) (status active) (actions speech_stack_state) (duration ?t))
	(item (name stack) (status nil))
	(pile (name simul) (first_stack $?pile1) (second_stack $?pile2))
	?f <- (stack_compare $?pile1)
	?f1 <- (stack_compare $?pile2)
	;?f2 <- (item (name speech_1))
	=>
	(retract ?f ?f1)
	(bind ?command (str-cat "I realize the two stacks are diferent, I will begin to explein what happened"))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
	;(modify ?f2 (image I_finish_the_simulation_I_start_to_execute_the_command))
)

(defrule exe-plan-speeched-stack
	?f <- (received ?sender command spg_say ?speech 1)
	?f2 <- (plan (name ?name) (number ?num-pln)(status active) (actions speech_stack_state))
	;?f3 <- (item (name stack))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	;(modify ?f3 (status review))
)

(defrule exe-plan-no-speeched-stack
	?f <- (received ?sender command spg_say ?speech 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech_stack_state))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-speech-generator
        (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator)(duration ?t))
        ?f1 <- (item (name speech_1)(status ?x&:(neq ?x said)) (image ?spg))
        =>
        (bind ?command (str-cat "" ?spg ""))
        (assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-af-speech-generator
        ?f <-  (received ?sender command spg_say ?spg 1)
        ?f1 <- (item (name speech_1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status said))
)

(defrule exe-plan-neg-speech-generator
        ?f <-  (received ?sender command spg_say ?spg 0)
        ?f1 <- (item (name speech_1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status said))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-backtracking
	(plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking) (duration ?t))
	?f <- (pile (name simul) (first_stack $?rest1 ?block_base_1) (second_stack $?rest2 ?block_base_2))
	=>
	(printout t "TEST STACKS: " $?rest1 ?block_base_1 " " $?rest2 ?block_base_2 " elements" crlf)
	(modify ?f (first_stack $?rest1) (second_stack $?rest2)(number 2))
	(assert (move ?block_base_1 cubestable first_stack 1))
	(assert (move ?block_base_2 cubestable second_stack 2))
	(assert (pop first_stack ?block_base_1))
	(assert (pop second_stack ?block_base_2))
)

(defrule exe-plan-backtracking-accomplish
	?f <- (received ?sender command cmd_make_backtraking ?conf 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	?f3 <- (item (name stack))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status review))
)

(defrule exe-plan-no-backtracking
	?f <- (received ?sender command cmd_make_backtraking ?conf 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; reglas para comprobar si las pilas cambiaron

(defrule transform_second_to_compare
	?f2 <- (stack_second $?pile)
	?f3 <- (pile (name simul) (number 0))
	=>
	(retract ?f2)
	(modify ?f3 (first_stack $?pile) (number 1))
	(assert (stack_compare $?pile))
)

(defrule transform_second_to_compare_two
	?f2 <- (stack_second $?pile)
	?f3 <- (pile (name simul) (number 1))
	=>
	(retract ?f2)
	(modify ?f3 (second_stack $?pile) (number 2))
	(assert (stack_compare $?pile))
)

(defrule compare_stacks_first_comparition
	?f2 <- (stack_compare $?pile2)
	(stack $?pile&:(eq $?pile $?pile2))
	?f <- (item (name stack) (status nil))
	?f3 <- (pile (name simul) (status nil))
	=>
	(modify ?f (status first_comparition))
	(retract ?f2)
	(modify ?f3 (status first_attemp))
	
)

(defrule compare_stacks_second_comparition
	?f2 <- (stack_compare $?pile2)
	(stack $?pile&:(eq $?pile $?pile2))
	?f <- (item (name stack) (status first_comparition))
	?f3 <- (pile (name simul) (status first_attemp))
	=>
	;(modify ?f (status review))
	(modify ?f (status equal))
	(retract ?f2)
	(modify ?f3 (status second_attemp))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; reglas para hacer un respaldo de las pilas

(defrule backup_stack
	?f2 <- (stack_origin $?pile)
	;(pile (name original) (status ?st&:(neq ?st second_attemp)))
	=>
	(retract ?f2)
	(assert (stack $?pile))
	(assert (stack_backup $?pile))
)

(defrule backup_stack_first_attemp
	?f <- (stack_backup $?pile)
	?f2 <- (pile (name original)(status nil))
	=>
	(retract ?f)
	(modify ?f2 (first_stack $?pile)(status first_attemp))
)

(defrule backup_stack_second_attemp
	?f <- (stack_backup $?pile)
	?f2 <- (pile (name original)(status first_attemp))
	=>
	(retract ?f)
	(modify ?f2 (second_stack $?pile)(status second_attemp))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
