;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;	18/12/2017			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-put-on-top
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions put_on_top ?block1 ?block2))
	=>
        (assert(goal (move ?block1)(on-top-of ?block2)))
	;(modify ?f (status accomplished))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
	(bind ?command (str-cat "Cubes configuration did not change"))
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
	(bind ?command (str-cat "I realize cubes configuration is different, I will explain what I think happened"))
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
	(bind ?command (str-cat "I realize cubes configuration is different, I start to explain what happened"))
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
	(bind ?command (str-cat "I realize cubes confuguration is different, I try to explain what happened"))
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

;(defrule exe-plan-backtracking
;	(plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking) (duration ?t))
;	?f <- (pile (name simul) (first_stack $?rest1 ?block_base_1) (second_stack $?rest2 ?block_base_2))
;	=>
;	(printout t "TEST STACKS: " $?rest1 ?block_base_1 " " $?rest2 ?block_base_2 " elements" crlf)
;	(modify ?f (first_stack $?rest1) (second_stack $?rest2)(number 2))
;	(assert (move ?block_base_1 cubestable first_stack 1))
;	(assert (move ?block_base_2 cubestable second_stack 2))
;	(assert (pop first_stack ?block_base_1))
;	(assert (pop second_stack ?block_base_2))
;)

(defrule exe-plan-backtracking-accomplish
	?f <- (received ?sender command cmd_make_backtraking ?conf 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	?f3 <- (item (name stack))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status review))
)

(defrule exe-plan-backtracking-no-stack-change
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking))
	?f3 <- (item (name stack))
	?f4 <- (stack no_change)
	=>
	(retract ?f4)
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
;;; habilitar o deshabilitar la simulacion
(defrule exe-plan-enable-simul
	(plan (name ?name) (number ?num-pln) (status active) (actions enable_simul ?flag)(duration ?t))
	=>
        (bind ?command (str-cat "simul " ?flag ""))
        (assert (send-blackboard ACT-PLN cmd_enable_simul ?command ?t 4))
)

(defrule exe-plan-enabled-simul
	?f <- (received ?sender command cmd_enable_simul simul ?flag 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions enable_simul ?flag))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-enabled-simul
	?f <- (received ?sender command cmd_enable_simul simul ?flag 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions enable_simul ?flag))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; reglas para hablitar el brazo
(defrule exe-plan-enable-arm
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions enable_arm ?obj) (duration ?t))
	?f1 <- (item (name ?obj)(attributes ?arm))
	?f2 <- (Arm (name ?arm))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (attributes pick))
	(modify ?f2 (status ready) (grasp ?obj))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Move simul
(defrule exe-plan-move-simul
	(plan (name ?name) (number ?num-pln) (status active) (actions move_simul ?actuator ?obj)(duration ?t))
	(item (name ?obj) (pose ?x ?y ?z))
	(Arm (name ?arm) (status ready) (bandera ?id) (grasp ?obj))	
	=>
        (bind ?command (str-cat "" ?obj "_simul " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
)

(defrule exe-plan-moved-simul
	?f <- (received ?sender command move_actuator ?obj ?x ?y ?z ?flag 1)
	?f1 <- (item (name ?obj))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions move_simul ?actuator ?obj))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-moved-simul
	?f <- (received ?sender command move_actuator ?obj ?x ?y ?z ?flag 0)
	?f1 <- (item (name ?obj))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions move_simul ?acuator ?obj))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; Drop simul
(defrule exe-plan-drop-simul
	(plan(name ?name) (number ?num-pln) (status active) (actions place_block_simul ?block1 ?block2) (duration ?t))
	(item (name ?block1))
	(item (name ?block2) (num ?tam) (pose ?x ?y ?z))
	(Arm (name ?arm) (status ready) (bandera ?flag) (grasp ?block1))
	=>
	(bind ?command (str-cat "simul " ?block1 " " ?flag " " ?block2 " " ?tam " " ?x " " ?y " " ?z ""))
	(assert (send-blackboard ACT-PLN drop ?command ?t 4))
)

(defrule exe-plan-droped-simul
	?f <- (received ?sender command drop simul ?block1 ?flag ?block2 ?tam ?x ?y ?z 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions place_block_simul ?block1 ?block2))
	?f3 <- (item (name ?block1))
	?f4 <- (item (name ?block2))
	?f5 <- (Arm (bandera ?flag))
	?f6 <- (item (name robot))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f6 (hands nil))
	(modify ?f3 (status droped) (pose ?x ?y ?z))
	(modify ?f5 (status nil)(grasp nil))
)

(defrule exe-plan-no-droped-simul
	?f <- (received ?sender command drop simul ?block1 ?flag ?block2 ?tam ?x ?y ?z 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions place_block_simul ?block1 ?block2))
	=>
	(retract ?f)
	(modify ?f1 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; Update stacks
(defrule exe-plan-update-stack
	(plan (name ?name) (number ?num-pln) (status active) (actions update_stack ?block1)(duration ?t))
	=>
	(bind ?command (str-cat "navigation update"))
	(assert (send-blackboard ACT-PLN update_stack ?command ?t 4))
)

(defrule exe-plan-updated-stack
	?f <- (received ?sender command update_stack ?act ?up 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions update_stack ?block1))
	?f3 <- (item (name ?block1))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status on-top))
)

(defrule exe-plan-no-updated-stack
	?f <- (received ?sender command update_stack ?act ?up 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions update_stack ?block1))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;; Backup cubes, restore cubes, update status

(defrule exe-plan-backup-cubes
	?f <- (plan (name ?name) (number ?num-pln)(status active) (actions backup_cubes ?block))
	(item (name ?block)(pose ?x ?y ?z) (attributes ?arm))
	?f2 <- (item (name ?blockexp) (image ?block))
	=>
	(modify ?f (status accomplished))
	(modify ?f2 (pose ?x ?y ?z) (attributes ?arm))
)

(defrule exe-plan-restore-cubes
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_cubes ?block))
	?f1 <- (item (name ?block))
	?f2 <- (item (name ?plockexp) (image ?block) (pose ?x ?y ?z) (attributes ?arm))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (pose ?x ?y ?z) (attributes ?arm))
)

(defrule exe-plan-update-status-object
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions update_status ?obj ?status))
	?f1 <- (item (name ?obj))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (status ?status))
)

(defrule exe-plan-update-status-pila
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions update_status ?pile ?status))
	?f1 <- (pile (name ?pile))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (status ?status))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;restore stacks
(defrule restore_stacks_equals
	?f <-(plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks))
	?f1 <-(pile (name original) (first_stack $?pile1) (second_stack $?pile2) (status nil))
	?f2 <- (stack $?pile3&:(eq $?pile1 $?pile3))
	?f3 <- (stack $?pile4&:(eq $?pile2 $?pile4))
	=>
	(modify ?f1 (status second_attemp))
	(modify ?f (status accomplished))
)

(defrule restore_stacks_first_different
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks))
	?f1 <- (pile (name original) (first_stack $?pile1) (second_stack $?pile2) (status nil))
	?f2 <- (stack $?pile3&: (neq $?pile1 $?pile3))
	?f3 <- (stack $?pile4&:(and (eq $?pile2 $?pile4) (neq $?pile3 $?pile4)))
	=>
	;(retract ?f2)
	(modify ?f (status accomplished))
	(modify ?f1 (status second_attemp))
	(assert (stack $?pile1))
)

(defrule restore_stacks_second_different
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks))
	?f1 <- (pile (name original) (first_stack $?pile1) (second_stack $?pile2) (status nil))
	?f2 <- (stack $?pile3&:(eq $?pile3 $?pile1))
	?f3 <- (stack $?pile4&:(and (neq $?pile3 $?pile4) (neq $?pile2 $?pile4)))
	=>
	(retract ?f3)
	(modify ?f (status accomplished))
	(modify ?f1 (status second_attemp))
	(assert (stack $?pile2))

)

(defrule restore_stacks_differents
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks))
	?f1 <- (pile (name original) (first_stack $?pile1) (second_stack $?pile2)(status nil))
	?f2 <- (stack $?pile3&:(and (neq $?pile1 $?pile3) (neq $?pile2 $?pile3)))
	?f3 <- (stack $?pile4&:(and (neq $?pile3 $?pile4) (neq $?pile1 $?pile4) (neq $?pile2 $?pile4)))
	=>
	(retract ?f2 ?f3)
	(modify ?f (status accomplished))
	(modify ?f1 (status second_attemp))
	(assert (stack $?pile1))
	(assert (stack $?pile2))
)

(defrule restore_stacks_differents_only_one
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions restore_stacks))
	?f1 <- (pile (name original) (first_stack $?pile1) (second_stack $?pile2) (status nil))
	?f2<- (stack $?pile3&:(and (neq $?pile1 $?pile3) (neq $?pile2 $?pile3)))
	(not (stack $?pile4&:(neq $?pile3 $?pile4)))
	=>
	(retract ?f2)
	(modify ?f (status accomplished))
	(modify ?f1 (status second_attemp))
	(assert (stack $?pile1))
	(assert (stack $?pile2))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; move block explain
(defrule exe-plan-put-on-top-explain
	(plan (name ?name) (number ?num-pln) (status active) (actions put_on_top_only_speech ?block1 ?block2))
	=>
	;(assert (goal_simul (move ?block1) (on-top-of ?block2)))
	(assert (goal_only_speech (move ?block1) (on-top-of ?block2)))
)

(defrule exe-plan-put-on-top-explained
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions put_on_top_only_speech ?block1 ?block2))
	?f1 <- (item (name ?block1) (attributes on-top ?block2))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (attributes nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; Reset cubes psitions
(defrule exe-plan-reset-cube-position
	(plan (name ?name) (number ?num-pln) (status active) (actions reset_cube_pos ?block)(duration ?t))
	(item (name ?block) (pose ?x ?y ?z))
	=>
	(bind ?command (str-cat "" ?block " " ?x " " ?y " " ?z))
	(assert (send-blackboard ACT-PLN reset_cube_pos ?command ?t 4))
)

(defrule exe-plan-reseted-cube-position
	?f <- (received ?sender command reset_cube_pos ?block ?x ?y ?z 1)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions reset_cube_pos ?block))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
)

(defrule exe-plan-no-reseted-cube-position
	?f <- (received ?sender command reset_cube_pos ?block ?x ?y ?z 0)
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions reset_cube_pos ?block))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
