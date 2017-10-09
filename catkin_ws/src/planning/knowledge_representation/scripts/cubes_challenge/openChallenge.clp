;********************************************************
;*							*
;*							*
;*							*
;*							*
;*			University of Mexico		*
;*			Julio Cesar Cruz Estrda		*
;*							*
;*			17/06/2016			*
;*							*
;********************************************************
;
; Scene:
;	1) The robot waits for the instruction
;	


(deffacts scheduled_cubes

	(name-scheduled challenge 1 2)
	
	(name-schedule init 1 2)
	(state (name init) (number 0)(duration 6000)(status active))
	;(condition (conditional if) (arguments world status saw)(true-state 2)(false-state 1)(name-scheduled init)(state-number 1))
	(cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensor)(to world)(name-scheduled init)(state-number 1))
	(plan_active no)

	;	STATE 1	
	
 ;
	;(state (name challenge) (number 1)(duration 6000)(status active))
	(condition (conditional if) (arguments world status saw)(true-state 2)(false-state 1)(name-scheduled challenge)(state-number 1))
	(cd-task (cd cmdWhatSee) (actor robot)(obj robot)(from sensor)(to world)(name-scheduled challenge)(state-number 1))


	;	STATE 2
	; Robot wait for presentations
	(state (name challenge) (number 2)(duration 6000))
	(condition (conditional if) (arguments presentation status accomplished)(true-state 3)(false-state 2)(name-scheduled challenge)(state-number 2))
	(cd-task (cd cmdPresentation) (actor robot)(obj robot)(from sensor)(to presentation)(name-scheduled challenge)(state-number 2))

	;	STATE 3
	;The robot wait for questions about the enviroment and describe it
	(state (name challenge) (number 3)(duration 6000))
	(condition (conditional if) (arguments enviroment status described)(true-state 4)(false-state 3)(name-scheduled challenge)(state-number 3))
	(cd-task (cd cmdEnviroment) (actor robot)(obj robot)(from sensor)(to enviroment)(name-scheduled challenge)(state-number 3))



	;	STATE 4
	; The robot wait for a user instruction "the instruction will modify the enviroment"
	(state (name challenge) (number 4)(duration 6000)(status active))
	(condition (conditional if) (arguments enviroment status described)(true-state 4)(false-state 3)(name-scheduled challenge)(state-number 3))
	(cd-task (cd cmdEnviroment) (actor robot)(obj robot)(from sensor)(to enviroment)(name-scheduled challenge)(state-number 3))	
	
	
	
	
)

(defrule exe_cmdSpeech_open
	
	?f1 <- (cd-task (cd cmdSpeech) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	?f2 <- (plan_active no)
	 =>
	(retract ?f1)
	(retract ?f2)
        (bind ?command (str-cat "" ?robot "Speech"))
        (assert (send-blackboard ACT-PLN cmd_speech ?command 6000 4))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; reglas para verificar que haya nuevos comandos en la cola de comandos "cmdQ"

(defrule speech_command
	?f <- (received ?sender command cmd_speech ?arg 1)
	=> 
	(retract ?f)
	;(assert (cd-task (cd interp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 2)))
        (printout t ?arg crlf)
	(assert (state (name challenge) (number 1)(duration 6000)(status active)))
	;(assert (plan_active yes))
)


(defrule no_speech_command
	?f <- (received ?sender command cmd_speech ?arg 0)
	=> 
	(retract ?f)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled init)(state-number 1)))
        (printout t "NO HAY COMANDOS" crlf)
	(assert (plan_active no))
)

(defrule speech_command_from_explain
	?f2 <- (explain negative)
	=> 
	(retract ?f2)
        (assert (received ACT-PLN command cmd_world take_order 1))
	
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;; wait for initial command "what do you see"



(defrule task-what-you-see
	(state (name challenge) (number 1)(duration 6000)(status active))	
	(cd-task (cd cmdWhatSee) (actor robot)(obj robot)(from sensor)(to ?world)(name-scheduled challenge)(state-number 1))        
	=>
	(assert (plan (name what_you_see) (number 1)(actions question_world ?world)(duration 6000)))
	(assert (finish-planner what_you_see 1))
)

(defrule exe-plan-what-you-see
	(plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world)(duration ?t))
 	;?f1 <- (item (name ?obj))
        =>
        (bind ?command (str-cat "" ?world ""))
        (assert (send-blackboard ACT-PLN cmd_world ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-what-you-saw
        ?f <-  (received ?sender command cmd_world what_see_no 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
	?f1 <-(item (name ?world))
        =>
        (retract ?f)
        (modify ?f2 (status active))
	;(modify ?f1 (status saw))	
)

(defrule exe-plan-what-you-saw-execute
        ?f <-  (received ?sender command cmd_world execute 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        ?f1 <-(item (name ?world))
        =>
        (retract ?f)
        (modify ?f2 (status active))   
)

(defrule exe-plan-no-what-you-saw
        ?f <-  (received ?sender command cmd_world what_see_yes 1)
        ?f1 <- (item (name ?world))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)

(defrule exe-plan-no-what-you-saw-person
        ?f <-  (received ?sender command cmd_world what_see_person 1)
        ?f1 <- (item (name ?world))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)

(defrule exe-plan-no-what-you-saw-obj
        ?f <-  (received ?sender command cmd_world what_see_obj 1)
        ?f1 <- (item (name ?world))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)

(defrule exe-plan-fake
	?f <- (received ?sender command cmd_world explain 1)
	?f1 <- (item (name ?world))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions question_world ?world))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)

;;;;;;;;;;;; Describe the world question

(defrule exe-plan-describe-world
        ?f <- (received ?sender command cmd_world describe_world 1)
        ?f1 <- (item (name peter)(status ?st1))
	?f2 <- (item (name john)(status ?st2))
        =>
	(retract ?f)
        (bind ?command (str-cat "peter " ?st1 " john " ?st2 ""))
        (assert (send-blackboard ACT-PLN cmd_describe ?command 6000 4))
)

(defrule modify-world-person
	?f <- (received ?sender command cmd_modify ?person1 ?st1 ?person2 ?st2 1)
	?f1 <- (item (name ?person1))
	?f2 <- (item (name ?person2))
	=>
	(retract ?f)
	(modify ?f1 (status ?st1))
	(modify ?f2 (status ?st2))
)

(defrule modify-world-object
	?f <- (received ?sender command cmd_wobj ?object ?st1 1)
	?f1 <- (item (name ?object))
	=>
	(retract ?f)
	(modify ?f1 (status ?st1) (possession nobody))
)


;;;;;;;;;;;; Objects and person questions

(defrule exe-plan-where
        ?f <- (received ?sender command cmd_world ?object 1)
	(item (name ?object) (status ?st1) (possession ?person))
        =>
	(retract ?f)
        (bind ?command (str-cat "" ?object " " ?st1 " " ?person))
        (assert (send-blackboard ACT-PLN cmd_where ?command 6000 4))
)

;(defrule exe-plan-where_milk
;        ?f <- (received ?sender command cmd_world milk 1)
;	(item (name milk) (status ?st1) (possession ?person))
;        =>
;	(retract ?f)
;        (bind ?command (str-cat "coffe " ?st1))
;        (assert (send-blackboard ACT-PLN cmd_where ?command 6000 4))
;)

;(defrule exe-plan-where_stevia
;        ?f <- (received ?sender command cmd_world stevia 1)
;	(item (name stevia) (status ?st1))
;        =>
;	(retract ?f)
;        (bind ?command (str-cat "stevia " ?st1))
;        (assert (send-blackboard ACT-PLN cmd_where ?command 6000 4))
;)




;;;;;;; take my order please

(defrule task-take-oreder
        ?f <- (received ?sender command cmd_world take_order 1)
        =>
	(retract ?f)
        (bind ?command (str-cat "take_order"))
        (assert (send-blackboard ACT-PLN cmd_order ?command 6000 4))
)

;;;;;;;; reglas para verificar que haya nuevos comandos en la cola de comandos "cmdQ"

(defrule speech_command_dos
	?f <- (received ?sender command cmd_order ?arg 1)
	=> 
	(retract ?f)
	(assert (interprete active))
        (printout t ?arg crlf)
)


(defrule no_speech_command_dos
	?f <- (received ?sender command cmd_order ?arg 0)
	=> 
	(retract ?f)
	(assert (received CLIPS command cmd_world what_see_yes 1))
        (printout t "NO HAY COMANDOS" crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;; se manda a Blackboard el comando para ejecutar el INTERPRETE
(defrule exe_interprete_dos
        ?f1 <- (interprete active)
	 =>
        (retract ?f1)
        (bind ?command (str-cat "robot Interpreta"))
        (assert (send-blackboard ACT-PLN cmd_int ?command 6000 4))
)

;;;;;;;;;;;;;;;;;;;;;; Explain the status of the object

(defrule exe-plan-happen
        ?f <- (received ?sender command cmd_world happen ?object 1)
        (item (name ?object) (status ?st1) (possession ?person))
        =>
        (retract ?f)
        (bind ?command (str-cat "" ?object " " ?st1 " " ?person))
        (assert (send-blackboard ACT-PLN cmd_happen ?command 6000 4))
)


;;;;;;;;;;;;;;;;;;;;;; Presentations


(defrule exe-plan-presentation
	(state (name challenge) (number 2)(duration 6000)(status active))	
	(cd-task (cd cmdPresentation) (actor robot)(obj robot)(from sensor)(to ?presentation)(name-scheduled challenge)(state-number 2))        
	=>
	(assert (plan (name presentations) (number 1)(actions first_source ?presentation)(duration 6000)))
	(assert (plan (name presentations) (number 2)(actions all_people ?presentation)(duration 6000)))
	(assert (finish-planner presentations 2))
)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;enviroment

(defrule exe-plan-enviroment
	(state (name challenge) (number 3)(duration 6000)(status active))	
	(cd-task (cd cmdEnviroment) (actor robot)(obj robot)(from sensor)(to ?envy)(name-scheduled challenge)(state-number 3))        
	=>
	(assert (plan (name presentations) (number 1)(actions first_source ?envy)(duration 6000)))
	(assert (plan (name presentations) (number 2)(actions all_people ?envy)(duration 6000)))
	(assert (finish-planner presentations 2))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Instruction to modify enviroment






