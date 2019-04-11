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
 	?f1 <- (item (name ?obj)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-found-object
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z ?arm 1)
 	    ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	    ?f3 <- (Arm (name ?arm))
        =>
        (retract ?f)
        ;(modify ?f2 (status accomplished))
        (modify ?f3 (status verify))
	    (modify ?f1 (pose ?x ?y ?z) (status finded));;;; modified for verify arm task		
)

(defrule exe-plan-no-found-object
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished)) ;performance for IROS
        ;(modify ?f2 (status active))//normal performance gpsr and eegpsr
)


(defrule exe-plan-move-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?obj)(duration ?t))
 	    (item (name ?obj) (pose ?x ?y ?z) )
        (Arm (name ?arm) (status ready)(bandera ?id) (grasp ?obj))
        =>
        (bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
)



(defrule exe-plan-moved-actuator
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	?f3 <- (item (name robot));;;;;;;;;; T1 test for quit grasp object subtask
	;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f3 (hands ?object));;;;; T1 test
	(modify ?f1 (status grabed));;;;;; T1 test
        ;(retract ?f3)
)

;fix this later
;(defrule exe-plan-no-moved-actuator
;        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
;        ?f1 <- (item (name ?object))
;        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
;        ;?f3 <- (wait plan ?name ?num-pln ?t)
;        =>
;        (retract ?f)
;        (modify ?f1 (name ?object))
;)

;;
(defrule exe-plan-no-move-grasp-object
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	?f3 <- (state (name ?plan) (status active) (number ?n))
	?f4 <- (state (name ?plan) (status inactive) (number ?n2&:(eq ?n2 (+ 1 ?n))))
	?f5 <- (cd-task (cd ?p&:(or (eq ?p phandover) (eq ?p pobjloc))) (name-scheduled ?plan) (state-number ?n2))
	?f6 <- (state (name ?plan) (status inactive) (number ?n3&:(eq ?n3 (+ 1 ?n2))))
	?f7 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f2 (status unaccomplished))
	(modify ?f3 (status unaccomplished))
	(modify ?f6 (status active))
	(modify ?f7 (status nil) (grasp nil))
)

(defrule exe-plan-no-move-grasp-object-two
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	?f3 <- (state (name ?plan) (status active) (number ?n))
	?f4 <- (state (name ?plan) (status inactive) (number ?n2&:(eq ?n2 (+ 2 ?n))))
	?f5 <- (cd-task (cd ?p&:(or (eq ?p phandover) (eq ?p pobjloc))) (name-scheduled ?plan) (state-number ?n2))
	?f6 <- (state (name ?plan) (status inactive) (number ?n3&:(eq ?n3 (+ 1 ?n2))))
	?f7 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f2 (status unaccomplished))
	(modify ?f3 (status unaccomplished))
	(modify ?f6 (status active))
	(modify ?f7 (status nil) (grasp nil))
)

(defrule exe-plan-no-move-grasp-object-no-handover
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	?f3 <- (state (name ?plan) (status active) (number ?n))
	?f4 <- (state (name ?plan) (status inactive) (number ?n2&:(eq ?n2 (+ 1 ?n))))
	?f5 <- (cd-task (cd ?p&:(and (neq ?p phandover) (neq ?p pobjloc))) (name-scheduled ?plan) (state-number ?n2))
	?f6 <- (item (name robot))
	?f7 <- (state (name ?plan) (status inactive) (number ?n3&:(eq ?n3 (+ 2 ?n))))
	?f8 <- (cd-task (cd ?pt&:(and (neq ?pt phandover) (neq ?pt pobjloc))) (name-scheduled ?plan) (state-number ?n3))
        ;?f3 <- (wait plan ?name ?num-pln ?t)
        ?f9 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f6 (hands ?object))
	(modify ?f1 (status grabed))
        (modify ?f9 (status nil) (grasp nil))
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
	;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
        (modify ?f3 (hands nil))
	    (modify ?f1 (status droped));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,,jc
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

;;;;;;;;;find person

(defrule exe-plan-find-person
        (plan (name ?name) (number ?num-pln)(status active)(actions find-person ?person ?place)(duration ?t))
 	?f1 <- (item (name ?person)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?person " " ?place ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-found-person-no-name
        ?f <-  (received ?sender command find_object person ?place ?x ?y ?z 1)
        ?f1 <- (item (name person))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-person person ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (pose ?x ?y ?z) (status went))      
)

(defrule exe-plan-no-found-person-no-name
        ?f <-  (received ?sender command find_object person ?place ?x ?y ?z 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-person person ?place))
        =>
        (retract ?f)
        (modify ?f2 (status active))   
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
