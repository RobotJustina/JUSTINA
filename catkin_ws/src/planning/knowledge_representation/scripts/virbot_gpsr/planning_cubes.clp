;********************************************************
;*                                                      *
;*      planning_cubes.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*                      Adrian Revueltas                *
;*                                                      *
;*                      2/5/14                          *
;*                                                      *
;********************************************************

(deftemplate goal (slot move)(slot on-top-of))
(deftemplate attempt (slot move)(slot on-top-of)(slot number))
(defglobal ?*plan_time* = 30000)


(deffacts initial-state
        (stack blockA blockB blockC)
        (stack blockD blockE blockF)
        ;(stack blockC)
        ;(stack blockF)
        ;(goal (move blockC)(on-top-of blockF))
        ;(get-initial-state stacks)
	(plan (name cubes) (number 0)(duration 0))

)



(defrule move-directly
        ?goal <- (goal (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
        ?stack-1 <- (stack ?block1 $?rest1)
        ?stack-2 <- (stack ?block2 $?rest2)
	(plan (name ?name) (number ?num))
	(not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal ?stack-1 ?stack-2)
        (assert (stack $?rest1))
        (assert (stack ?block1 ?block2 $?rest2))
        (printout t ?block1 " will be moved on top of " ?block2 "." crlf)
        (assert (plan (name ?name) (number (+ 1 ?num))(actions find-object ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 2 ?num))(actions find-object ?block2)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 3 ?num))(actions move manipulator ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 4 ?num))(actions grab manipulator ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 5 ?num))(actions move manipulator ?block2 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 6 ?num))(actions drop manipulator ?block1 )(duration ?*plan_time*)) )
	(assert (attempt (move ?block1) (on-top-of ?block2)(number (+ 6 ?num) )))
)



(defrule move-to-table
        ?goal <- (goal (move ?block1) (on-top-of cubestable))
        ?stack-1 <- (stack ?block1 $?rest)
	(plan (name ?name) (number ?num))
        (not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal ?stack-1)
        (assert (stack $?rest))
        (printout t ?block1 " will be moved on top of the table." crlf)
        (assert (plan (name ?name) (number (+ 1 ?num))(actions find-object ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 2 ?num))(actions find-object cubestable)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 3 ?num))(actions move manipulator ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 4 ?num))(actions grab manipulator ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 5 ?num))(actions move manipulator cubestable )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 6 ?num))(actions drop manipulator ?block1 )(duration ?*plan_time*)) )
	(assert (attempt (move ?block1) (on-top-of cubestable)(number (+ 6 ?num)) ))
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

