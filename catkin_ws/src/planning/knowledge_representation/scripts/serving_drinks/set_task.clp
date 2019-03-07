;********************************************************
;*                                                      *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cruz                      *
;*                                                      *
;*                      2/13/17                         *
;*                                                      *
;********************************************************

;;;;;;; reglas para insertar tareas una a una


(defrule set_task_command_two
        ?f <- (received ?sender command cmd_set_task ?user ?action_type ?param1 ?param2 ?step 1)
        ;(item (name ?param) (zone ?zone))
        (plan_name ?plan)
        =>
        (retract ?f)
        (printout t "Se obtuvo tarea: " ?action_type crlf)
        (assert(task ?plan ?action_type ?param1 ?param2 ?step))

)

(defrule set_task_command_one
        ?f <- (received ?sender command cmd_set_task ?user ?action_type ?param ?step 1)
        ;(item (name ?param) (zone ?zone))
        (plan_name ?plan)
        =>
        (retract ?f)
        (printout t "Se obtuvo tarea: " ?action_type crlf)
        (assert(task ?plan ?action_type ?param ?step))


)

(defrule set_task_command_no_param
        ?f <- (received ?sender command cmd_set_task ?user ?action_type ?step 1)
        ;(item (name ?param) (zone ?zone))
        (plan_name ?plan)
        =>
        (retract ?f)
        (printout t "Se obtuvo tarea: " ?action_type crlf)
        (assert(task ?plan ?action_type ?step))


)

(defrule set_no_task_command_arena
        ?f <- (received ?sender command cmd_set_task ?param 0)
        ?f1 <- (num_steps ?steps)
        ?f2 <- (state (name ?plan) (number 1)(duration 6000))
        ?f3 <- (plan_name ?plan)
        (num_intentos ?nint)
        ?f4 <- (intento ?intento&:(< ?intento ?nint))
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f3)
        (retract ?f4)
        (assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
        (assert (name-scheduled ?plan 1 ?steps))
        (assert (task ?plan update_object_location algo arena ?steps))
        (modify ?f2 (status active))
        (assert (intento (+ ?intento 1)))
)

(defrule set_no_task_command_exitdoor
        ?f <- (received ?sender command cmd_set_task ?param 0)
        ?f1 <- (num_steps ?steps)
        ?f2 <- (state (name ?plan) (number 1)(duration 6000))
        ?f3 <- (plan_name ?plan)
        (num_intentos ?nint)
        ?f4 <- (intento ?intento&:(eq ?intento ?nint))
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f3)
        (retract ?f4)
        (assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
        (assert (name-scheduled ?plan 1 ?steps))
        (assert (task ?plan update_object_location algo exitdoor ?steps))
        (modify ?f2 (status active))
        (assert (intento 1))
)

(defrule set_no_task_command_cero_steps
        ?f <- (received ?sender command cmd_set_task ?param 0)
        ?f1 <- (num_steps 0)
        ?f2 <- (plan_name ?plan)
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f2)
        (assert (cd-task (cd cmdSpeech) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 1)))
        (printout t "NO HAY TAREAS" crlf)
)

;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;; Reglas para insertar hechos iniciales
;;;; (plan_name ?name)

(defrule set_plan_name
        ?f1 <- (received ?sender command cmd_set_task ?name ?steps 1)
	;?f2 <- (plan_active ?n)
        =>
        (retract ?f1 )
        (assert (plan_name ?name))
        (assert (num_steps (+ ?steps 1)))
)

