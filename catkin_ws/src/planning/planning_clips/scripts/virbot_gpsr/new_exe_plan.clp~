

 

;;; creamos la regla de ejecucion de cracion de estados
(defrule exe_create_state
	?f <- (received ?sender command cmd_int ?arg 1)
	=> 
	(retract ?f)
	(assert (tasks true))
	(assert (cd-task (cd get_task) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 2)))
        (printout t "Ejecutando modulo crear estados" crlf)
	
)

(defrule task_update_object_location
	?f <- (received ?sender command cmd_task ?user update_object_location ?param ?step 1)
	(item (name ?param) (zone ?zone))
	=>
	(printout t "Update object location task" crlf)
	
	(retract ?f)
       	;ACTIONS
	
	(assert (state (name cubes) (number (+ ?step 1))(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?zone)(true-state (+ ?step 1))(false-state ?step)(name-scheduled cubes)(state-number ?step)))
	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?zone)(name-scheduled cubes)(state-number ?step)))

	
	(assert (into (name cubes)(number ?step)(next (+ ?step 1))(status accomplished)))
)


(defrule task_get_object
	?f <- (received ?sender command cmd_task ?user get_object ?param ?step 1)
	=>
	(printout t "Get Object task" crlf)
	(retract ?f)
	(assert (state (name cubes) (number (+ ?step 1))(duration 6000)))
	; ACTIONS
	(assert (objetive get_obj task_get ?param ?step))
	;(assert (plan (name task_get) (number 1)(actions find-object ?param)(duration 6000)))
	;(assert (plan (name task_get) (number 2)(actions move manipulator ?param)(duration 6000)))
	;(assert (plan (name task_get) (number 3)(actions grab manipulator ?param)(duration 6000)))
	;(assert (into (name task_get)(number ?step)(next (+ ?step 1))(plan 3)))
	;(assert (finish-planner task_get 3))
)

(defrule find_person_in_room
	?f <- (received ?sender command cmd_task ?user find_person_in_room ?param ?step 1)
	(item (name ?param)(zone ?zone))
	=>	
	(printout t " Find Person in room task" crlf)
	(retract ?f)
	(assert (state (name cubes) (number (+ ?step 1))(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?zone)(true-state (+ ?step 1))(false-state ?step)(name-scheduled cubes)(state-number ?step)))
	

	(assert (plan (name task_find) (number 1)(actions find-object ?param)(duration 6000)))
	(assert (into (name task_find)(number ?step)(next (+ ?step 1))(plan 1)))
	(assert (finish-planner task_find 1))

	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from kitchen)(to ?zone)(name-scheduled cubes)(state-number ?step)))

)

(defrule task_handover_object
	?f <- (received ?sender command cmd_task ?user handover_object ?param ?step 1)
	=>
	(printout t " Handover object task" crlf)
	(retract ?f)
	(assert (state (name cubes) (number (+ ?step 1))(duration 6000)))
	
	(assert (plan (name task_handover) (number 1)(actions move manipulator person)(duration 6000)))
	(assert (plan (name task_handover) (number 2)(actions drop manipulator ?param)(duration 6000)))
	(assert (into (name task_handover)(number ?step)(next (+ ?step 1))(plan 2)))
	(assert (finish-planner task_handover 2))
	
)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;; reglas para subplanes ;;;;;;;;;;;;;;;;;;;;


(defrule exe-plan-ask-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?obj)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone))
        =>
        (bind ?command (str-cat  "" ?obj ""))
        (assert (send-blackboard ACT-PLN ask_for ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-asked-actuator
        ?f <-  (received ?sender command ask_for ?object ?zone 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object))
        ?f3 <- (item (name robot))
	;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f1 (zone ?zone))
        ;(retract ?f4)
)

(defrule exe-plan-no-asked-actuator
        ?f <-  (received ?sender command ask_for ?object ?zone 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

;;regla para moverse

(defrule exe-plan-go-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions go_to ?object)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone))
        =>
        (bind ?command (str-cat  "" ?object " " ?zone ""))
        (assert (send-blackboard ACT-PLN goto ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-went-actuator
        ?f <-  (received ?sender command goto ?object ?zone 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to ?object))
        ?f3 <- (item (name robot))
	;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
)

(defrule exe-plan-no-go-actuator
        ?f <-  (received ?sender command goto ?object ?zone 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

;;;;;regla para status de puerta
(defrule exe-plan-status-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object)(duration ?t))
        ?f1 <- (item (name ?object) (zone ?zone))
	?f2 <- (item (name ?obj2) (possession ?zone))
        =>
        (bind ?command (str-cat  "" ?obj2 ""))
        (assert (send-blackboard ACT-PLN status_object ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-stated-actuator
        ?f <-  (received ?sender command status_object ?obj2 open 1)
        ?f1 <- (item (name ?object) (zone ?zone))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
	;?f4 <- (wait plan ?name ?num-pln ?t)
	?f5<- (item (name ?obj2) (possession ?zone))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f5 (status open))
        ;(retract ?f4)
)

(defrule exe-plan-no-stated-flase
        ?f <-  (received ?sender command status_object ?obj2 ?status 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
	
)

(defrule exe-plan-no-stated-true
        ?f <-  (received ?sender command status_object ?obj2 closed 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
	
)

;;;;;;;;;;;;;;;;;;;;;;;regla cuando ya no hay mas tareas en la cola;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule no_task_pile
	?f <- (received ?sender command cmd_task ?arg 0)
	?f1 <- (state (name cubes) (number 1)(duration 6000))
	=>
	(assert (tasks false))
	;(modify ?f1 (status active))
)

