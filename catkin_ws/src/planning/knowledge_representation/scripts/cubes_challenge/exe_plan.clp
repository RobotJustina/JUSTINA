;********************************************************
;*                                                      *
;*      exe_plan.clp	                                *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*                      Adrian Revueltas                *
;*                      Julio Cesar Cruz Estrada        *
;*                                                      *
;*                      2/10/17                          *
;*                                                      *
;********************************************************


(defrule exe-plan
	(finish-planner ?name ?num_pln)
        ?f <- (plan (name ?name) (number ?num&:(neq num 0))(status inactive))
        (not (plan (name ?name) (number ?num1&:( < ?num1 ?num))(status ?status&:(or (eq ?status active) (eq ?status inactive  )))) )
        =>
	(modify ?f (status active))
)


(defrule exe-plan-find-object
        (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?obj)(duration ?t))
 	?f1 <- (item (name ?obj)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-found-object
        ?f <-  (received ?sender command find_object ?object ?x&:(neq ?x 0) ?y&:(neq ?y 0) ?z&:(neq ?z 0) ?arm 1)
 	    ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	?f3 <- (Arm (name ?arm))
        =>
        (retract ?f)
        ;(modify ?f2 (status accomplished))
        (modify ?f3 (status verify))
	(modify ?f1 (pose ?x ?y ?z)(status finded));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;jc		
)

(defrule exe-plan-no-found-object-exception
        ?f <-  (received ?sender command find_object ?object ?x&:(eq ?x 0) ?y&:(eq ?y 0) ?z&:(eq ?z 0) ?arm 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	;?f6 <- (plan (name ?name) (number ?num-pln3&:(eq ?num-pln3 (+ 1 ?num-pln)))(status inactive)(actions find-object ?block2))
	?f6 <- (plan (name ?name) (number ?num-pln3)(status active)(actions onnly-find-object ?block2))
        ?f4 <- (plan (name ?name) (number ?num-pln1)(status inactive)(actions move ?actuator ?object))
        ?f5 <- (plan (name ?name) (number ?num-pln2)(status inactive)(actions grab ?actuator ?object))
	?f7 <- (plan (name ?name) (number ?num-pln4&:(eq ?num-pln4 (+ 1 ?num-pln3)))(status inactive)(actions place-block ?object ?block2))
	?f8 <- (plan (name ?name) (number ?num-pln5)(status inactive) (actions pile ?object ?block2))
	?f9 <- (plan (name ?name) (number ?num-pln6) (status inactive) (actions align_with_point ?block2 ?ori_frame ?dest_frame))
	?f3 <- (state (name ?plan) (status active) (number ?n))
        =>
        (retract ?f)
        (modify ?f2 (status unaccomplished))
        (printout t "TEST FOR NEW NO OBJECT EXCEPTION" crlf)
        (modify ?f1 (status nil))
        (assert (plan_obj ?object))
        (assert (plan_person cube_aux))
        (assert (fuente found))
        (assert (cd-task (cd disp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
	(modify ?f3 (status unaccomplished))
	(retract ?f4)
	(retract ?f5)
	;(retract ?f6)
	(retract ?f7)
	(retract ?f8)
	(retract ?f9)
)

(defrule exe-plan-no-found-object-exception-second
	?f <- (received ?sender command find_object ?object ?x&:(eq ?x 0) ?y&:(eq ?y 0) ?z&:(eq ?z 0) ?arm 1)
	?f1 <- (item (name ?block2))
	?fobj <- (item (name ?object))
	?f8 <- (plan (name ?name) (number ?num-pln0)(status accomplished)(actions find-object ?object))
	;?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?block2))
	?f2 <- (plan (name ?name) (number ?num-pln)(status active) (actions only-find-object ?block2))
	?f3 <- (plan (name ?name) (number ?num-pln1)(status accomplished)(actions move ?actuator ?object))
	?f4 <- (plan (name ?name) (number ?num-pln2)(status accomplished)(actions grab ?actuator ?object))
	?f5 <- (plan (name ?name) (number ?num-pln3)(status inactive)(actions place-block ?object ?block2))
	?f6 <- (plan (name ?name) (number ?num-pln4)(status inactive)(actions pile ?object ?block2))
	?f9 <- (plan (name ?name) (number ?num-pln5)(status accomplished) (actions align_with_point ?block2 ?ori_frame ?dest_frame))
	?f7 <- (state (name ?plan) (status active) (number ?n))
	=>
	(retract ?f)
	(modify ?f2 (status unaccomplished))
	(printout t "NO FOUND OBJECT EXCEPTION" crlf)
	(modify ?f1 (status nil))
	(modify ?fobj (status nil))
	(assert (plan_obj ?block2))
	(assert (plan_person cube_aux))
	(assert (fuente found))
	(assert (cd-task (cd disp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
	(modify ?f7 (status unaccomplished))
	(retract ?f3)
	(retract ?f4)
	(retract ?f5)
	(retract ?f6)
	(retract ?f9)
)

(defrule exe-plan-no-found-object-exception-third
	?f <- (received ?sender command find_object ?object ?x&:(eq ?x 0) ?y&:(eq ?y 0) ?z&:(eq ?z 0) ?arm 1)
	?f1 <- (item (name ?object))
	?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	?f3 <- (plan (name ?name) (number ?num-pln1)(status inactive)(actions move ?actuator ?object))
	?f4 <- (plan (name ?name) (number ?num-pln2)(status inactive)(actions grab ?actuator ?object))
	?f5 <- (plan (name ?name) (number ?num-pln3&:(eq ?num-pln3 (+ 1 ?num-pln2)))(status inactive)(actions drop object ?object))
	?f6 <- (state (name ?plan) (status active) (number ?n))
	?f7 <- (plan (name ?name) (number ?num-pln4)(status inactive)(actions pile ?object))
	=>
	(retract ?f)
	(modify ?f2 (status unaccomplished))
	(printout t "EXCEPTION NO FOUND OBJECT" crlf)
	(modify ?f1 (status nil))
	(assert (plan_obj ?object))
	(assert (plan_person cube_aux))
	(assert (fuente found))
        (assert (cd-task (cd disp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
	(modify ?f6 (status unaccomplished))
	(retract ?f3)
	(retract ?f4)
	(retract ?f5)
	(retract ?f7)
)

;;;;;;;;;;;;;;;;;;;; reglas para buscar objeto que no se va graspear (Only-find-object task)

(defrule exe-plan-find-object-no-grasp
        (plan (name ?name) (number ?num-pln)(status active)(actions only-find-object ?obj)(duration ?t))
 	?f1 <- (item (name ?obj)(status ?x&:(neq ?x finded) $?rest_1))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-blackboard ACT-PLN only_find_object ?command ?t 4))
	
)

(defrule exe-plan-found-object-no-grasp
	?f <- (received ?sender command only_find_object ?object ?x ?y ?z ?arm 1)
	?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions only-find-object ?object))
	=>
	(retract ?f)
	(modify ?f1 (status finded) (pose ?x ?y ?z))
	(modify ?f2 (status accomplished))	
)

(defrule exe-plan-no-found-object-no-grasp 
        ?f <-  (received ?sender command only_find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions only-find-object ?object))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;; ciclo para eliminar todas las subtareas de nombre ?name( solo se haprobado con la subtarea task_get)

(defrule exe-delate-task-loop
        (finish-planner ?name ?num)
        ?f <- (plan (name ?name) (number ?num-pln) (status inactive))
        ?f2 <- (delate_task ?name ?num-pln&:(<= ?num-pln ?num))
        =>
        (retract ?f)
        (retract ?f2)
        (assert (delate_task ?name (+ ?num-pln 1)))
)

(defrule exe-delate-task-end
        ?f2 <- (finish-planner ?name ?num)
        ?f <- (delate_task ?name ?num-pln&:(> ?num-pln ?num))
        =>
        (retract ?f)
        (retract ?f2)
    )
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-no-found-object
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
        ;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status active))
)


(defrule exe-plan-move-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?obj)(duration ?t))
 	(item (name ?obj) (pose ?x ?y ?z) )
        (Arm (name ?arm) (status ready)(bandera ?id) (grasp ?obj))
        =>
        (bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)



(defrule exe-plan-moved-actuator
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f3)
)

;fix this later
(defrule exe-plan-no-moved-actuator
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
        ;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)



(defrule exe-plan-grab-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions grab ?actuator ?obj)(duration ?t))
        ?f1 <- (item (name ?obj))
        =>
        (bind ?command (str-cat "" ?actuator " " ?obj ""))
        (assert (send-blackboard ACT-PLN grab ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)


(defrule exe-plan-grabed-actuator
        ?f <-  (received ?sender command grab ?actuator ?object 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions grab ?actuator ?object))
	?f3 <- (item (name robot))
	;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
	(modify ?f3 (hands ?object))
	(modify ?f1 (status grabed)) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;jc 
)


(defrule exe-plan-no-grabed-actuator
        ?f <-  (received ?sender command grab ?actuator ?object 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions grab ?actuator ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)



(defrule exe-plan-drop-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?obj)(duration ?t))
        ?f1 <- (item (name ?obj))
        ?f2 <- (Arm (name ?arm) (status ready) (bandera ?flag) (grasp ?obj))
        =>
        (bind ?command (str-cat "" ?actuator " " ?obj " " ?flag ""))
        (assert (send-blackboard ACT-PLN drop ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-droped-actuator
        ?f <-  (received ?sender command drop ?actuator ?object ?flag 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?object))
        ?f3 <- (item (name robot))
        ?f4 <- (Arm (bandera ?flag))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f3 (hands nil))
	(modify ?f1 (status droped));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,,jc
        (modify ?f4 (status nil) (grasp nil))
)

(defrule exe-plan-no-droped-actuator
        ?f <-  (received ?sender command drop ?actuator ?object ?flag 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

;;;;;; place block on block rules

(defrule exe-plan-place-block-on-block
	(plan (name ?name) (number ?num-pln)(status active) (actions place-block ?block1 ?block2) (duration ?t))
	?f1 <- (item (name ?block1))
	?f2 <- (item (name ?block2)(num ?tam))
	?f3 <- (Arm (name ?arm) (status ready) (bandera ?flag) (grasp ?block1))
	=>
	(bind ?command (str-cat "block " ?block1 " " ?flag " " ?block2 " " ?tam))
	(assert (send-blackboard ACT-PLN drop ?command ?t 4))

)

(defrule exe-plan-placed-block-on-block
	?f <- (received ?sender command drop ?actuator ?block1 ?flag ?block2 ?tam 1)
	?f1 <- (item (name ?block1))
	?f2 <- (item (name ?block2))
	?f3 <- (plan (name ?name) (number ?num-pln)(status active) (actions place-block ?block1 ?block2) (duration ?t))
	?f4 <- (item (name robot))
	?f5 <- (Arm (bandera ?flag))
	=>
	(retract ?f)
	(modify ?f3 (status accomplished))
	(modify ?f4 (hands nil))
	(modify ?f1 (status droped))
	(modify ?f5 (status nil) (grasp nil))
)

(defrule exe-plan-no-placed-block-on-block
	?f <- (received ?sender command drop ?actuator ?block1 ?flag ?block2 ?tam 0)
	?f1 <- (item (name ?block1))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active)(actions place-block ?block1 ?block2) (duration ?t))
	?f3 <- (item (name robot))
	=>
	(retract ?f)
	(modify ?f1 (name ?block1))
)

;;;;;;;;;;;;;

;;;;;;;;;;;;;
