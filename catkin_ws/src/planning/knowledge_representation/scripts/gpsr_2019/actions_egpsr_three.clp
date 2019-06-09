;;;;;;;;;;;;;;;;;
;;;;; EGPSR
;;;;;; Julio Cruz
;;;;; 7-06-2019
;;;;;;;;;;;;;


(defrule task_get_rpose_category
	?f <- (task ?plan get_rpose_object object ?place ?rpose ?category ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	(item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_rpose_obj ?place ?rpose ?category ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)
;;;;;;;;;;;;;;;;;;
(defrule plan_get_rpose_object
	?goal <- (objetive get_rpose_obj ?name ?place ?rpose ?category ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Relpos Object Task" crlf)
	(bind ?confirmation(str-cat "For this task I need your help, Do you want to help me, say justina yes or justina no"))
	(bind ?speech(str-cat "If you dont help me, I can not do the task"))
	(assert (plan (name ?name) (number 1)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions make_task ?name)(actions_num_params 3 3)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions ask_for object ?place)(duration 6000)))
	;(assert (plan (name ?name) (number 4)(actions go_to object)(duration 6000)))
	;(assert (plan (name ?name) (number 5)(actions attend object)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-rpose-object ?place ?rpose ?category)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task_neg ?name object grabed)(actions_num_params 5 5)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions speech-anything ?speech)(duration 6000)))
	;(assert (plan (name ?name) (number 5)(actions make_task ?name object finded)(actions_num_params 6 6)(duration 6000)))
	;(assert (plan (name ?name) (number 6)(actions move manipulator )(duration 6000))) 
	(assert (plan (name ?name) (number 6)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 6))
)
;;;;;;;;;;;;;;;;;;
(defrule exe_scheduled-get-rpose-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_rpose_obj ?place ?rpose ?category ?step)
	=>
	(retract ?f1)
	(assert (objetive get_rpose_obj task_get_rpose_obj ?place ?rpose ?category ?step))
)

