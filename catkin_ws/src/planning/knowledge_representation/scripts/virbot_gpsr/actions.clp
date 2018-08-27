;********************************************************
;*                                                      *
;*      schedule_cubes.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      03/02/2016                      *
;*                                                      *
;********************************************************

;;;;;;;

;find_person_in_room			params 1	Ready
;wait_for_user_instruction
;update_object_location			params 2	Ready (crackers bathroom)
;get_object				params 1	Ready
;put_object_in_location			params 2	
;save_position				params 1	(current_loc) 
;deliver_in_position			params 2	exemple (pringles current_loc) 
;handover_object			params 1	Ready



(defrule task_update_object_location
	?f <- (task ?plan update_object_location ?param1 ?param2 ?step)
	?f1 <- (item (name ?param2))
	;?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Object location task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?param2)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	;(modify ?f2 (zone frontexit))
		
)

(defrule task_put_object_in_location
	?f <- (task ?plan put_object_in_location ?param1 ?param2 ?step)
	?f1 <- (item (name ?param1))
	=>
	(retract ?f)
	(printout t "Put object in location task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pobjloc) (actor robot)(obj robot)(from ?param1)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	
		
)

(defrule task_deliver_in_position
	?f <- (task ?plan deliver_in_position ?param1 ?param2 ?step)
	?f1 <- (item (name ?patam1))
	;?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Deliver in position" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pobjloc) (actor robot)(obj robot)(from ?param1)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	;(assert (condition (conditional if) (arguments robot zone kitchen)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	;(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to kitchen)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	;(modify ?f2 (zone frontexit))
	
		
)



(defrule task_find_person_in_room
	?f <- (task ?plan find_person_in_room ?person&:(or (eq ?person person) (eq ?person man)) ?param1 ?step)
	?f1 <- (item (name ?param1))
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Find person in room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments person status went)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfindperson) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)


(defrule task_find_specific_person_in_room
	?f <- (task ?plan find_person_in_room ?person&:(neq ?person person) ?place ?step)
	?f1 <- (item (name ?place))
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Find Specific person in room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?person status went)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfindspcperson) (actor robot)(obj robot)(from ?person)(to ?place)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_object
	?f <- (task ?plan get_object ?param1 ?place ?step)
	?f1 <- (item (name ?param1)(type Objects))
        (item (name ?place))
	=>
	(retract ?f)
	(printout t "Get object" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pgetobj) (actor robot)(obj robot)(from ?place)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;;;;;;
	(modify ?f1 (status nil))	
)

(defrule task_get_object_man
	?f <- (task ?plan get_object man ?place ?step)
	?f1 <- (item (name man)(type Person))
	=>
	(retract ?f)
	(printout t "Follow MAN" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments man status followed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pgetobjman) (actor robot)(obj robot)(from ?place)(to man)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;;;;;;
	(modify ?f1 (status nil))	
)

(defrule task_get_object_man_guide
        ?f <- (task ?plan get_object man_guide ?place ?step)
        ?f1 <- (item (name man_guide)(type Person))
        =>
        (retract ?f)
        (printout t "GUIDE MAN" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments man_guide status followed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pgetobjman) (actor robot)(obj robot)(from ?place)(to man_guide)(name-scheduled ?plan)(state-number ?step)))
        ;;;;;;;;;;;
        (modify ?f1 (status nil))       
)

(defrule task_save_position
	?f <- (task ?plan save_position ?param1 ?step)
	?f1 <- (item (name ?param1))
	?f2 <- (item (name robot) (zone ?zone))
	=>
	(retract ?f)
	(printout t "Save position" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?param1)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	;(modify ?f1 (status nil))
	(modify ?f2 (zone frontexit))
)


(defrule task_handover_object
	?f <- (task ?plan handover_object ?param1 ?step)
	?f1 <- (item (name ?param1))
	=>
	(retract ?f)
	(printout t "Handover object" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd phandover) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
)

(defrule task_wait_for_user_instruction
	?f <- (task ?plan wait_for_user_instruction ?num_question ?question_task ?step)
	?f1 <- (item (name ?num_question))
	?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Wait for user instruction" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?num_question status ask)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pquestion) (actor robot)(obj robot)(from ?question_task)(to ?num_question)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	(modify ?f2 (zone frontexit))

	
	
)





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;  SPLIT IN SUBTAREAS


(defrule plan_ask_for
        ?goal <- (objetive get_obj ?name ?param ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions ask_for ?param ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?param)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?param)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?param)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions move manipulator ?param)(duration 6000)))
	;(assert (plan (name ?name) (number 6)(actions grab manipulator ?param)(duration 6000)));T1 test
	;(assert (into (name ?name)(number ?step)(next (+ ?step 1))(plan 6)))
	(assert (finish-planner ?name 5))
)


(defrule plan_find_person
        ?goal <- (objetive find_person ?name ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?param)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-person person ?param)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions go_to_person person)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_handover_obj
        ?goal <- (objetive handover_obj ?name ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	;(assert (plan (name ?name) (number 1)(actions move manipulator person)(duration 6000)))
	(assert (plan (name ?name) (number 1)(actions drop person ?param)(duration 6000)))
	(assert (finish-planner ?name 1))
)


(defrule plan_put_obj_in_loc
        ?goal <- (objetive put_obj_loc ?name ?param1 ?param2 ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?param2)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions ask_for ?param1)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions go_to ?param1)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions attend ?param1)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions move manipulator ?param1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions drop object ?param1)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_answer_question
        ?goal <- (objetive answer_question ?name ?question_task ?num_question ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Answer question Task" crlf)
	(assert (plan (name ?name) (number 1)(actions answer_question ?num_question ?question_task)(duration 6000)))
	(assert (finish-planner ?name 1))
)


(defrule plan_get_obj_man
        ?goal <- (objetive get_obj_man ?name ?place ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions find-object-man ?param ?place)(duration 6000)))
	(assert (finish-planner ?name 1))
)


(defrule plan_find_person_spc
        ?goal <- (objetive find_spc_person ?name ?person ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-person specific ?person ?place)(duration 6000)))
	(assert (finish-planner ?name 2))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RULES BEFORE SPLIT IN SUBTAREAS

(defrule exe_scheduled-get
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgetobj) (actor ?robot)(obj ?robot)(from ?place)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive get_obj task_get ?param1 ?place ?step))
)

(defrule exe_scheduled-find
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfindperson) (actor ?robot)(obj ?robot)(from ?from)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_person task_find ?param1 ?step))
)

(defrule exe_scheduled-handover
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd phandover) (actor ?robot)(obj ?robot)(from ?from)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive handover_obj task_handover ?param1 ?step))
)

(defrule exe_scheduled-put-object-in-location
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pobjloc) (actor ?robot)(obj ?robot)(from ?param1)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive put_obj_loc task_pobjloc ?param1 ?param2 ?step))
)

(defrule exe_wait_for_user_instruction
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pquestion) (actor ?robot)(obj ?robot)(from ?question_task)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive answer_question task_aquestion ?question_task ?param2 ?step))
)


(defrule exe_get_object_man
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgetobjman) (actor ?robot)(obj ?robot)(from ?place)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive get_obj_man task_get_man ?place ?param2 ?step))
)

(defrule exe_scheduled-find-specific
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfindspcperson) (actor ?robot)(obj ?robot)(from ?person)(to ?place)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_spc_person task_find_spc ?person ?place ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



