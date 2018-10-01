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
        (intento 1)
        (num_intentos 1)
	(plan_active no)
        (num_places 0)
        (visit_places 0)

	;;for speech the number of orders in eegpsr cat2 montreal
	(num_order 1)
	(order _)
	
	;;;for speech the person description in eegpsr cat2 montreal
	(person_description _)
	
	
	;(state (name cubes) (number 4)(duration 6000)(status active))
)

(defrule exe_cmdSpeech
	
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
	(assert (cd-task (cd interp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 2)))
        (printout t ?arg crlf)
	;(assert (plan_active yes))
)


(defrule no_speech_command
	?f <- (received ?sender command cmd_speech ?arg 0)
	=> 
	(retract ?f)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY COMANDOS" crlf)
	(assert (plan_active no))
)

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
	?f <- (received ?sender command cmd_int ?plan 1)
	=> 
	(retract ?f)
	(assert (cd-task (cd conf) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
        ;(printout t "Inicia modulo crear PLAN" crlf)
	(assert (plan_conf ?plan))
)

(defrule no_int_command
	?f <- (received ?sender command cmd_int ?arg 0)
	=> 
	(retract ?f)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "Error, volver a mandar un COMANDO para el Robot" crlf)
	(assert (plan_active no))
	
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
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 3)))
        (printout t "Inicia modulo crear PLAN" crlf)
	(assert (num_steps (+ ?steps 1)))
	(assert (plan_name ?plan))
)

(defrule no_conf_command
	?f <- (received ?sender command cmd_conf ?arg 0)
	=> 
	(retract ?f)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "Este COMANDO no se va a ejecutar" crlf)
	(assert (plan_active no))
	
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

(defrule task_command_fifth
	?f <- (received ?sender command cmd_task ?user ?action_type ?param1 ?param2 ?param3 ?param4 ?param5 ?step 1)
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	(assert (cd-task (cd get_task) (actor robot) (obj robot) (from sensor) (to status) (name-scheduled cubes) (state-number 3)))
	(assert (task ?plan ?action_type ?param1 ?param2 ?param3 ?param4 ?param5 ?step))
)

(defrule task_command_fourth
	?f <- (received ?sender command cmd_task ?user ?action_type ?param1 ?param2 ?param3 ?param4 ?step 1)
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	(assert (cd-task (cd get_task) (actor robot) (obj robot) (from sensors) (to status) (name-scheduled cubes) (state-number 3)))
	(assert (task ?plan ?action_type ?param1 ?param2 ?param3 ?param4 ?step))
)

(defrule task_command_three
	?f <- (received ?sender command cmd_task ?user ?action_type ?param1 ?param2 ?param3 ?step 1)
	(plan_name ?plan)
	=>
	(retract ?f)
	(printout t "Se obtuvo tarea: " ?action_type crlf)
	(assert (cd-task (cd get_task) (actor robot) (obj robot) (from sensors) (to status) (name-scheduled cubes) (state-number 3)))
	(assert (task ?plan ?action_type ?param1 ?param2 ?param3 ?step))
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

(defrule no_task_command_arena
	?f <- (received ?sender command cmd_task ?param 0)
	?f1 <- (num_steps ?steps)
	?f2 <- (state (name ?plan) (number 1)(duration 6000))
	?f3 <- (plan_name ?plan)
        (num_intentos ?nint)
        ?f4 <- (intento ?intento&:(< ?intento ?nint))
	?f5 <- (item (name robot))
        => 
	(retract ?f)
	(retract ?f1)
	(retract ?f3)
        (retract ?f4)
	(assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
	(assert (name-scheduled ?plan 1 (+ ?steps 1)))
	(assert (task ?plan update_object_location algo current_loc ?steps))
        (assert (task ?plan speech_generator speech_1 (+ ?steps 1)))
	(modify ?f2 (status active))
	(modify ?f5 (zone frontexit))
        (assert (intento (+ ?intento 1)))
)

(defrule no_task_command_exitdoor
        ?f <- (received ?sender command cmd_task ?param 0)
        ?f1 <- (num_steps ?steps)
        ?f2 <- (state (name ?plan) (number 1)(duration 6000))
        ?f3 <- (plan_name ?plan)
        (num_intentos ?nint)
        ?f4 <- (intento ?intento&:(eq ?intento ?nint))
	?f5 <- (item (name robot))
        => 
        (retract ?f)
        (retract ?f1)
        (retract ?f3)
        (retract ?f4)
        (assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
        (assert (name-scheduled ?plan 1 (+ ?steps 2)))
        (assert (task ?plan update_object_location algo current_loc ?steps))
	(assert (task ?plan speech_generator speech_2 (+ ?steps 1)))
	(assert (task ?plan finish_clips finish_clips (+ ?steps 2)))
        (modify ?f2 (status active))
	(modify ?f5 (zone frontexit))
        (assert (intento 1))
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
        =>
        (modify ?f1 (status inactive))
	(assert (plan_active no))
	
	(retract ?f3)
)

(defrule exe_scheduled-if-conditional-false-location-final-task
        (item (name ?object) (zone ?status_object))
        (name-scheduled ?name ?ini ?st)
        ?f3 <- (condition (conditional if) (arguments ?object zone ?status&:(neq ?status ?status_object) )
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
     	=>
        (modify ?f1 (status inactive))
	(assert (plan_active no))
	
	(retract ?f3)
)

(defrule exe_scheduled-if-conditional-true-status-final-task
        (item (name ?object) (status $?status_object))
        (name-scheduled ?name ?ini ?st)
        ;(condition (conditional if) (arguments ?object status $?status&:(eq $?status $?status_object)) 
        ?f3 <-(condition (conditional if) (arguments ?object status $?status_object) 
                (true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
        ?f1 <- (state (name ?name) (number ?st&:(neq ?st ?ts))(status active))
	
        =>
	(modify ?f1 (status inactive))
	(assert (plan_active no))
	
	(retract ?f3)
)



(defrule exe_scheduled-if-conditional-false-status-final-task
	(item (name ?object) (status $?status_object))
    	(name-scheduled ?name ?ini ?st)
	?f3 <- (condition (conditional if) (arguments ?object status $?status&:(neq $?status $?status_object) )
		(true-state ?ts)(false-state ?fs)(name-scheduled ?name)(state-number ?st))
	?f1 <- (state (name ?name) (number ?st&:(neq ?st ?fs))(status active))
	=>
	(modify ?f1 (status inactive))
	(assert (plan_active no))
	
	(retract ?f3)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;; Manejar numero de comandos que va recibir por prueba;;;;
;;;;;;;;;;;; Por ejemplo Si son 3 comandos debe viajar 2 veces a la Arena y la última vez al exitdoor;;;;;;;




