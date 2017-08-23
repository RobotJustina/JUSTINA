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
;*                      2/2/17                          *
;*                                                      *
;********************************************************

;(deftemplate plan
        ;(field name 
                 ;(type SYMBOL)
                 ;(default nil)
        ;) 
        ;(field number
                ;(type NUMBER)
                ;(default 1)
        ;)
        ;(multifield actions
                ;(type SYMBOL)
        ;)
        ;(field duration
                ;(type NUMBER)
                ;(default 1)
        ;)
        ;(field status
                 ;(type SYMBOL)
                 ;(default inactive)
        ;)
;)


;(deftemplate item
        ;(field type
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(field name
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(multifield status
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(multifield attributes
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(multifield pose
                 ;(type NUMBER)
                 ;(default 0 0 0)
        ;)
        ;(field grasp
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(field zone
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(field possession
                 ;(type SYMBOL)
                 ;(default nobody)
        ;)
        ;(field image
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(field script
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(field num
                 ;(type NUMBER)
                 ;(default 1)
        ;)
        ;(field shared
                 ;(type SYMBOL)
                 ;(default false)
        ;)
        ;(multifield zones
                 ;(type SYMBOL)
                 ;(default nil)
        ;)
        ;(multifield hands
                 ;(type SYMBOL)
                 ;(default nil)
        ;)

;)



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
        ?f4 <- (plan (name ?name) (number ?num-pln1)(status inactive)(actions move ?actuator ?object))
        ?f5 <- (plan (name ?name) (number ?num-pln2)(status inactive)(actions grab ?actuator ?object))
	?f3 <- (state (name ?plan) (status active) (number ?n))
        =>
        (retract ?f)
        (modify ?f2 (status unaccomplished))
        (printout t "TEST FOR NEW NO OBJECT EXCEPTION" crlf)
        (modify ?f1 (status nil))
        (assert (plan_obj ?object))
        (assert (plan_person john));;;;;;hardcode
        (assert (fuente found))
        (assert (cd-task (cd disp) (actor robot)(obj robot)(from sensors)(to status)(name-scheduled cubes)(state-number 6)))
	(modify ?f3 (status unaccomplished))
	(retract ?f4)
	(retract ?f5)
        ;(assert (delate_task ?name 1))
        ;(assert (delate_task task_find_spc 1))
        ;(assert (delate_task task_handover 1))

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

