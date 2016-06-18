;********************************************************
;*                                                      *
;*      exe_plan.clp	                                *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*                      Adrian Revueltas                *
;*                                                      *
;*                      2/13/14                         *
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
 	?f1 <- (item (name ?obj))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-found-object
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z 1)
 	?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f3)
	(modify ?f1 (pose ?x ?y ?z)(status finded));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;jc		
)

(defrule exe-plan-no-found-object
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z 0)
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
        =>
        (bind ?command (str-cat "" ?actuator " " ?x " " ?y " " ?z ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)



(defrule exe-plan-moved-actuator
        ?f <-  (received ?sender command move_actuator ?actuator ?x ?y ?z 1)
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
        ?f <-  (received ?sender command move_actuator ?actuator ?x ?y ?z 0)
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
        =>
        (bind ?command (str-cat "" ?actuator " " ?obj ""))
        (assert (send-blackboard ACT-PLN drop ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-droped-actuator
        ?f <-  (received ?sender command drop ?actuator ?object 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?object))
        ?f3 <- (item (name robot))
	;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
        (modify ?f3 (hands nil))
	(modify ?f1 (status droped));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,,jc
)

(defrule exe-plan-no-droped-actuator
        ?f <-  (received ?sender command drop ?actuator ?object 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

