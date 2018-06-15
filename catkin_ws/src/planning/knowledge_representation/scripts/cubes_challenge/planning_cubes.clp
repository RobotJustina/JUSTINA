;********************************************************
;*                                                      *
;*      planning_cubes.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*                      Cruz Estrada Julio Cesar        *
;*                                                      *
;*                      3/10/17                         *
;*                                                      *
;********************************************************

(deftemplate goal (slot move)(slot on-top-of))
(deftemplate attempt (slot move)(slot on-top-of)(slot number))
(defglobal ?*plan_time* = 30000)


(deffacts initial-state
        ;(stack blockA blockB blockC)
        ;(stack blockD blockE blockF)
        ;(stack blockC)
        ;(stack blockF)
        ;(goal (move blockC)(on-top-of blockF))
        ;(get-initial-state stacks)
	(plan (name cubes) (number 0)(duration 0))

)

(defrule dont-move-directly
	(declare (salience 1000))
	?goal <- (goal (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
	?stack-1 <- (stack $?top ?block1 ?block2 $?bottom)
	(plan (name cubes) (number ?num))
	(not (plan (name cubes) (number ?num1&:( > ?num1 ?num))))
	=>
	(retract ?goal)
	(printout t "" ?block1 " is already on top of " ?block2 "." crlf)
	(bind ?speech(str-cat "" ?block1 " is already on top of " ?block2))
	(assert (plan (name cubes) (number (+ 1 ?num)) (actions speech-anything ?speech) (duration 6000)))
	(assert (plan (name cubes) (number (+ 2 ?num)) (actions pile ?block1 ?block2 upgrade_state)))
	(assert (finish-planner cubes 2))
)

(defrule dont-move-on-top-of-table
	(declare (salience 1000))
	?goal <- (goal (move ?block1) (on-top-of cubestable))
	?stack-1 <- (stack $?top ?block1)
	(plan (name cubes) (number ?num))
	(not (plan (name cubes) (number ?num1&:( > ?num1 ?num))))
	=>
	(retract ?goal)
	(printout t "" ?block1 " is already on top of the table." crlf)
	(bind ?speech (str-cat "" ?block1 " is already on top of the table"))
	(assert (plan (name cubes) (number (+ 1 ?num)) (actions speech-anything ?speech) (duration 6000)))
	(assert (plan (name cubes) (number (+ 2 ?num)) (actions pile ?block1 cubestable upgrade_state)))
	(assert (finish-planner cubes 2))
	
)

(defrule move-directly
        ?goal <- (goal (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
        ?stack-1 <- (stack ?block1 $?rest1)
        ?stack-2 <- (stack ?block2 $?rest2)
	(plan (name cubes) (number ?num))
	(not (plan (name cubes) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        ;(assert (stack $?rest1))
        ;(assert (stack ?block1 ?block2 $?rest2))
        (printout t ?block1 " will be move on top of " ?block2 "." crlf)
	;(assert (plan (name cubes) (number 1 )(actions go_to ?block1)(duration 6000)))
	(assert (plan (name cubes) (number 1 )(actions update_status ?block1 nil)(duration 6000)))
	(assert (plan (name cubes) (number 2 )(actions update_status ?block2 nil)(duration 6000)))
	(assert (plan (name cubes) (number 3 )(actions attend ?block1)(duration 6000)))
	(assert (plan (name cubes) (number 4 )(actions align_with_point ?block1 map base_link)(duration 6000)))
        (assert (plan (name cubes) (number 5 )(actions find-object ?block1)(duration 6000)))
        ;(assert (plan (name cubes) (number 3 )(actions only-find-object ?block2)(duration 6000)))
        (assert (plan (name cubes) (number 6 )(actions move manipulator ?block1 )(duration 6000)))
        (assert (plan (name cubes) (number 7 )(actions grab manipulator ?block1 )(duration 6000)))
        ;(assert (plan (name ?name) (number (+ 5 ?num))(actions move manipulator ?block2 )(duration ?*plan_time*)))
	(assert (plan (name cubes) (number 8 ) (actions align_with_point ?block2 map base_link)(duration 6000)))
	(assert (plan (name cubes) (number 9 ) (actions only-find-object ?block2)(duration 6000)))
        (assert (plan (name cubes) (number 10 )(actions place-block ?block1 ?block2)(duration 6000)))
	(assert (plan (name cubes) (number 11 ) (actions only-find-object ?block1)(duration 6000)))
	(assert (plan (name cubes) (number 12)(actions pile ?block1 ?block2)(duration 6000)))
	(assert (finish-planner cubes 12))
	;(assert (attempt (move ?block1) (on-top-of ?block2)(number 8 )))
)



(defrule move-to-table
        ?goal <- (goal (move ?block1) (on-top-of cubestable))
        ?stack-1 <- (stack ?block1 $?rest)
	(plan (name cubes) (number ?num))
        (not (plan (name cubes) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        ;(assert (stack $?rest))
        (printout t ?block1 " will be move on top of the table." crlf)
	;(assert (plan (name cubes) (number 1 )(actions go_to ?block1)(duration 6000)) )
	(assert (plan (name cubes) (number 1)(actions update_status ?block1 nil)(duration 6000)))
	(assert (plan (name cubes) (number 2 )(actions attend ?block1)(duration 6000)) )
	(assert (plan (name cubes) (number 3 ) (actions align_with_point ?block1 map base_link)(duration 6000)))
        (assert (plan (name cubes) (number 4 )(actions find-object ?block1)(duration 6000)) )
        ;(assert (plan (name ?name) (number (+ 2 ?num))(actions find-object cubestable)(duration ?*plan_time*)) )
        (assert (plan (name cubes) (number 5 )(actions move manipulator ?block1 )(duration 6000)) )
        (assert (plan (name cubes) (number 6 )(actions grab manipulator ?block1 )(duration 6000)) )
        ;(assert (plan (name ?name) (number (+ 5 ?num))(actions move manipulator cubestable )(duration ?*plan_time*)) )
        (assert (plan (name cubes) (number 7 )(actions drop object ?block1)(duration 6000)) )
	(assert (plan (name cubes) (number 8 )(actions only-find-object ?block1)(duration 6000)))
	(assert (plan (name cubes) (number 9 )(actions pile ?block1)(duration 6000)))
	(assert (finish-planner cubes 9))
	;(assert (attempt (move ?block1) (on-top-of cubestable)(number 7 ) ))
)


(defrule clear-upper-block
         (declare (salience -9200))
        (goal (move ?block1))
        (stack ?top  $? ?block1 $?)
	=>
        (assert (goal (move ?top)(on-top-of cubestable)))
)



(defrule clear-lower-block
        (declare (salience -9100))
        (goal (on-top-of ?block1))
        (stack ?top  $? ?block1 $?)
        =>
        (assert (goal (move ?top)(on-top-of cubestable)))
)


(defrule finish-plan
	(declare (salience -10000))
        (plan (name ?name) (number ?num&:(neq ?num 0)))
        (not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
        ?f <- (plan (name ?name) (number 0))
        =>
	(retract ?f)
	(assert (finish-planner ?name ?num))
)




(defrule accomplish-plan
	(plan (name ?name) (number ?num) (status accomplished))
	?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
	?f3 <- (item (type Objects) (name ?block1)) 
	?f4 <- (item (type Objects) (name ?block2)) 
	=>
	(retract ?f2)
	(modify ?f3 (status on-top-of ?block2))
	(modify ?f4 (status down-of ?block1))
)



(defrule accomplish-plan-furniture
        (plan (name ?name) (number ?num) (status accomplished))
        ?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
        ?f3 <- (item (type Objects) (name ?block1))
        ?f4 <- (item (type Furniture) (name ?block2)(status $?data))
        =>
        (retract ?f2)
        (modify ?f3 (status on-top-of ?block2))
        (modify ?f4 (status $?data ?block1))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule add-cubes
	?f <- (cmd_insert cube ?obj ?x ?y ?z 1)
	?f1 <- (item (name ?obj))
	=>
	(retract ?f)
	(modify ?f1 (pose ?x ?y ?z))
)

(defrule add-cubes-two
	?f<-(cmd_insert cube ?obj ?x ?y ?z ?arm 1)
	?f1 <- (item (name ?obj))
	?f2 <- (Arm (name ?arm))
	=>
	(retract ?f)
	(modify ?f1 (pose ?x ?y ?z)(attributes ?arm))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;
