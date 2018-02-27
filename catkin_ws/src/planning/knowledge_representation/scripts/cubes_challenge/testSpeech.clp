;********************************************************
;*							*
;*	schedule_cubes.clp				*
;*							*
;*							*
;*			University of Mexico		*
;*			Julio Cesar Cruz Estrda		*
;*							*
;*			03/02/2016			*
;*							*
;********************************************************
;
; Scene:
;	1) The robot waits for the instruction
;	


(deffacts scheduled_cubes

	;(name-scheduled cubes 1 5)
	;;; para iniciar las pruebas con el interprete
	; The robot recieve a command
	; DURATION
	;(state (name cubes) (number 1)(duration 6000)(status active))
	; INPUTS
	;(condition (conditional if) (arguments robot zone frontentrance)(true-state 1)(false-state 2)(name-scheduled cubes)(state-number 1))
	; ACTIONS
	(cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1))

	(plan_active no)

	(equal_stack 0)	
	
	;(state (name cubes) (number 4)(duration 6000)(status active))
)

;(defrule exe_cmdSpeech
;	
;	?f1 <- (cd-task (cd cmdSpeech) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
;	?f2 <- (plan_active no)
;	 =>
;	(retract ?f1)
;	(retract ?f2)
;        (bind ?command (str-cat "" ?robot "Speech"))
;        (assert (send-blackboard ACT-PLN cmd_speech ?command 6000 4))
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; reglas para verificar que haya nuevos comandos en la cola de comandos "cmdQ"

;(defrule speech_command
;	?f <- (received ?sender command cmd_speech ?arg 1)
;	=> 
;	(retract ?f)
;	(assert (cd-task (cd interp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 2)))
 ;       (printout t ?arg crlf)
;	;(assert (plan_active yes))
;)


;(defrule no_speech_command
;	?f <- (received ?sender command cmd_speech ?arg 0)
;	=> 
;	(retract ?f)
;	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
;       (printout t "NO HAY COMANDOS" crlf)
;	(assert (plan_active no))
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;; se manda a Blackboard el comando para ejecutar el INTERPRETE
(defrule exe_interprete
        ?f1 <- (cd-task (cd interp) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	 =>
        (retract ?f1)
        (bind ?command (str-cat "" ?robot " Interpreta"))
        (assert (send-blackboard ACT-PLN cmd_int ?command 6000 4))
)


;;; respuesta del comando "cmd_int"
(defrule int_command
	?f <- (received ?sender command cmd_int ?plan ?obj ?person 1)  ;;; Add ?obj and ?person
	=> 
	(retract ?f)
	(assert (cd-task (cd disp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
        ;(printout t "Inicia modulo crear PLAN" crlf)
	(assert (plan_conf ?plan))
        (assert (plan_obj ?obj))
        (assert (plan_person ?person))
        (assert (fuente interprete))
)


(defrule no_int_command
	?f <- (received ?sender command cmd_int ?arg 0)
	=> 
	(retract ?f)
	;(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        ;(printout t "Error, volver a mandar un COMANDO para el Robot" crlf)
	;(assert (plan_active no))
        (assert (explain negative))
	
)

(defrule no_int_command_two_params
        ?f <- (received ?sender command cmd_int ?robot ?interpreta 0)
        => 
        (retract ?f)
        ;(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "Error, volver a mandar un COMANDO para el Robot" crlf)
        ;(assert (plan_active no))
        (assert (explain negative))
        
)

(defrule no_int_command_cero_params
        ?f <- (received ?sender command cmd_int 0)
        => 
        (retract ?f)
        ;(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "Error, volver a mandar un COMANDO para el Robot" crlf)
        ;(assert (plan_active no))
        (assert (explain negative))
        
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;; reglas para confirmar si el objeto se encuentra disponible o no

(defrule exe_disponible
        ?f1 <- (cd-task (cd disp) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
        ?f3 <- (plan_obj ?obj)
        ?f4 <- (plan_person ?block2)
        ?f5 <- (fuente ?f)
        (item (name ?obj) (status ?obj_st $?rest_1) (possession ?pos)(image ?im1))
        (item (name ?block2) (status ?prs_st $?rest_2)(image ?im))
        =>
        (retract ?f1)
        (retract ?f3)
        (retract ?f4)
        (retract ?f5)
        ;(bind ?command (str-cat "" ?obj_st " " ?prs_st " " ?f " " ?pos))
	(bind ?command (str-cat "" ?im1 " " ?im " " ?f " " ?block2))
        (assert (send-blackboard ACT-PLN cmd_disp ?command 6000 4))
)


(defrule disponible
        ?f <- (received ?sender command cmd_disp ?obj_st ?prs_st ?fuente ?pos 1)
        => 
        (retract ?f)
        (assert (cd-task (cd conf) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
)

(defrule no_disponible
        ?f <- (received ?sender command cmd_disp ?obj_st ?prs_st ?fuente ?pos 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        => 
        (retract ?f)
        (modify ?f2 (status active))
)

(defrule happen
        ?f <- (received ?sender command cmd_happen ?obj_st ?prs_st ?fuente 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
        => 
        (retract ?f)
        (modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; se manda el mensaje para pedir CONFIRMACION

(defrule exe_confirmation
        ?f1 <- (cd-task (cd conf) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	?f2 <- (plan_conf ?plan)
	 =>
        (retract ?f1)
	(retract ?f2)
        (bind ?command (str-cat "" ?plan))
        (assert (send-blackboard ACT-PLN cmd_conf ?command 6000 4))
)

;;; respuesta del comando "cmd_conf"
(defrule conf_command
	?f <- (received ?sender command cmd_conf ?plan ?steps 1)
	=> 
	(retract ?f)
	;(assert (cd-task (cd explain) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
        (printout t "Inicia modulo crear PLAN" crlf)
	(assert (num_steps (+ ?steps 1)))
	(assert (plan_name ?plan))
)

(defrule no_conf_command
	?f <- (received ?sender command cmd_conf ?arg 0)
	=> 
	(retract ?f)
	;(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "Este COMANDO no se va a ejecutar" crlf)
	;(assert (plan_active no))
	(assert (explain negative))
	
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  exe explain the plan
(defrule exe_explain_what_you_do
	?f1 <- (cd-task (cd explain) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	=>
	(retract ?f1)
	(bind ?command (str-cat "" ?robot " explain task"))
        (assert (send-blackboard ACT-PLN cmd_explain ?command 6000 4))
)

(defrule yes_execute_command
	?f <- (received ?sender command cmd_explain ?plan ?steps 1)
	=> 
	(retract ?f)
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
)

(defrule no_execute_command
	?f <- (received ?sender command cmd_explain ?plan ?steps 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions question_world ?world))
	=> 
	(retract ?f)
        (printout t "Este PLAN no se va a ejecutar" crlf)
        (modify ?f2 (status active))
	
)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;; se manda llamar a blackboard para obtener la siguiente tarea

(defrule exe_get_task
	?f1 <- (cd-task (cd get_task) (actor ?robot)(obj ?robot)(from ?from)(to ?to)(name-scheduled ?name)(state-number ?num-state))
	=>
	(retract ?f1)
	(bind ?command (str-cat "" ?robot " Get Task"))
        (assert (send-blackboard ACT-PLN cmd_task ?command 6000 4))
)

(defrule task_command_two
	?f <- (received ?sender command cmd_task ?user ?action_type ?param1 ?param2 ?step 1)
	;(item (name ?param) (zone ?zone))
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
	(assert(task ?plan ?action_type ?param1 ?param2 ?step))
	
)

(defrule task_command_one
	?f <- (received ?sender command cmd_task ?user ?action_type ?param ?step 1)
	;(item (name ?param) (zone ?zone))
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
	(assert(task ?plan ?action_type ?param ?step))
	
       	
)

(defrule task_command_no_param
	?f <- (received ?sender command cmd_task ?user ?action_type ?step 1)
	;(item (name ?param) (zone ?zone))
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
	(assert(task ?plan ?action_type ?step))
	
       	
)

(defrule no_task_command
	?f <- (received ?sender command cmd_task ?param 0)
	?f1 <- (num_steps ?steps)
	?f2 <- (state (name ?plan) (number 1)(duration 6000))
	?f3 <- (plan_name ?plan)
	=> 
	(retract ?f)
	(retract ?f1)
	(retract ?f3)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
	(assert (name-scheduled ?plan 1 ?steps))
	(assert (task ?plan update_object_location algo dining_room ?steps));;;; here I changed dining_room instead exitdoor
	(modify ?f2 (status active))
)

(defrule no_task_command_cero_steps
	?f <- (received ?sender command cmd_task ?param 0)
	?f1 <- (num_steps 0)
	?f2 <- (plan_name ?plan)
	=> 
	(retract ?f)
	(retract ?f1)
	(retract ?f2)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
	(printout t "NO HAY TAREAS" crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;; reglas para volver inactive la ultima task del plan

(defrule exe_scheduled-if-conditional-true-location-final-task
        (item (name ?object) (zone ?status_object))
        (name-scheduled ?name ?ini ?st)
        ?f3 <- (condition (conditional if) (arguments ?object zone ?status&:(eq ?status ?status_object) )
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?ts))(status active))
        ?f2 <- (plan (name ?name2) (number ?num-pln)(status active)(actions question_world ?world));; only for open challenge
        =>
        (modify ?f1 (status inactive))
	;(assert (plan_active no)) this fact was commented only for open challenge
	(modify ?f2 (status active)); for open chalenge
	(retract ?f3)
)

(defrule exe_scheduled-if-conditional-false-location-final-task
        (item (name ?object) (zone ?status_object))
        (name-scheduled ?name ?ini ?st)
        ?f3 <- (condition (conditional if) (arguments ?object zone ?status&:(neq ?status ?status_object) )
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
        ?f2 <- (plan (name ?name2) (number ?num-pln)(status active)(actions question_world ?world));; only for open challenge
     	=>
        (modify ?f1 (status inactive))
	;(assert (plan_active no)) this fact was commented only for open challenge
	(modify ?f2 (status active)); for open challenge
	(retract ?f3)
)

(defrule exe_scheduled-if-conditional-true-status-final-task
        (item (name ?object) (status $?status_object))
        (name-scheduled ?name ?ini ?st)
        ;(condition (conditional if) (arguments ?object status $?status&:(eq $?status $?status_object)) 
        ?f3 <-(condition (conditional if) (arguments ?object status $?status_object) 
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?ts))(status active))
	?f2 <- (plan (name ?name2) (number ?num-pln)(status active)(actions question_world ?world));; only for open challenge
        =>
	(modify ?f1 (status inactive))
	;(assert (plan_active no)) this fact was commented only for open challenge
	(modify ?f2 (status active)); for open challenge
	(retract ?f3)
)



(defrule exe_scheduled-if-conditional-false-status-final-task
	(item (name ?object) (status $?status_object))
    	(name-scheduled ?name ?ini ?st)
	?f3 <- (condition (conditional if) (arguments ?object status $?status&:(neq $?status $?status_object) )
		(true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
	?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
        ?f2 <- (plan (name ?name2) (number ?num-pln)(status active)(actions question_world ?world));; only for open challenge
	=>
	(modify ?f1 (status inactive))
	;(assert (plan_active no)) this fact was commented only for open challenge
	(modify ?f2 (status active));;; for open challenge
	(retract ?f3)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




