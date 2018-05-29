;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;	28/05/2018			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; simulacion de acciones ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftemplate goal_only_speech (slot move)(slot on-top-of))
(deftemplate attempt_only_speech (slot move)(slot on-top-of)(slot number))

(deffacts initial-only-speech-state
	(plan (name cubes_only_speech) (number 0)(duration 0))
)

(defrule only-speech-dont-move-directly
        (declare (salience 1000))
	?goal <- (goal_only_speech (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
	?stack-1 <- (stack $?top ?block1 ?block2 $?bottom)
	(plan (name cubes_only_speech) (number ?num))
	(not (plan (name cubes_only_speech) (number ?num1&:( > ?num1 ?num))))
	=>
	(retract ?goal)
	(printout t "" ?block1 " is already on top of " ?block2 "." crlf)
	(bind ?speech (str-cat "" ?block1 " is already on top of " ?block2 ""))
	(assert (plan (name cubes_only_speech) (number 1)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_only_speech) (number 2)(actions pile_simul ?block1 ?block2 upgrade_state)(duration 6000)))
	(assert (finish-planner cubes_only_speech 2))
)

(defrule only-speech-dont-move-to-table
	(declare (salience 1000))
	?goal <- (goal_only_speech (move ?block1) (on-top-of cubestable))
	?stack-1 <- (stack $?top ?block1)
	(plan (name cubes_only_speech) (number ?num))
	(not (plan (name cubes_only_speech) (number ?num1&:(> ?num1 ?num))))
	
	=>
	(retract ?goal)
	(printout t "" ?block1 " is already on top of the table." crlf )
	(bind ?speech (str-cat "" ?block1 " is already on top of the table"))
	(assert (plan (name cubes_only_speech) (number 1) (actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_only_speech) (number 2) (actions pile_simul ?block1 cubestable upgrade_state)(duration 6000)))
	(assert (finish-planner cubes_only_speech 2))
)

(defrule only-speech-move-directly
        ?goal <- (goal_only_speech (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
        ?stack-1 <- (stack ?block1 $?rest1)
        ?stack-2 <- (stack ?block2 $?rest2)
	(plan (name cubes_only_speech) (number ?num))
	(not (plan (name cubes_only_speech) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        (printout t ?block1 " will be moved on top of " ?block2 "." crlf)
	(bind ?speech (str-cat "" ?block1 " will be moved on top of " ?block2 ""))
	(assert (plan (name cubes_only_speech) (number 1)(actions speech-anything ?speech)(duration 6000)))
	;(assert (plan (name cubes_simul) (number 2)(actions enable_arm ?block1)(duration 6000)))
        ;(assert (plan (name cubes_simul) (number 3)(actions move_simul manipulator ?block1 )(duration 6000)))
        ;(assert (plan (name cubes_simul) (number 4 )(actions grab manipulator ?block1 )(duration 6000)))
        ;(assert (plan (name cubes_simul) (number 5 )(actions place_block_simul ?block1 ?block2)(duration 6000)))
	(assert (plan (name cubes_only_speech) (number 2)(actions pile_simul ?block1 ?block2)(duration 6000)))
	(assert (finish-planner cubes_only_speech 2))
	;(assert (attempt (move ?block1) (on-top-of ?block2)(number (+ 1 ?num) )))
)

(defrule only-speech-move-to-table
        ?goal <- (goal_only_speech (move ?block1) (on-top-of cubestable))
        ?stack-1 <- (stack ?block1 $?rest)
	(plan (name cubes_only_speech) (number ?num))
        (not (plan (name cubes_only_speech) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        (printout t ?block1 " will be moved on top of the table." crlf)
	(bind ?speech (str-cat "" ?block1 " will be moved on top of the table"))
	(assert (plan (name cubes_only_speech) (number 1)(actions speech-anything ?speech)(duration 6000)))
	;(assert (plan (name cubes_only_speech) (number 2)(actions enable_arm ?block1)(duration 6000)))
        ;(assert (plan (name cubes_only_speech) (number 3)(actions move_simul manipulator ?block1 )(duration 6000)))
        ;(assert (plan (name cubes_only_speech) (number 4)(actions grab manipulator ?block1 )(duration 6000)) )
        ;(assert (plan (name cubes_only_speech) (number 5)(actions place_block_simul ?block1 ?block1)(duration 6000)) )
	(assert (plan (name cubes_only_speech) (number 2)(actions pile_simul ?block1)(duration 6000)))
	(assert (finish-planner cubes_only_speech 2))
	;(assert (attempt (move ?block1) (on-top-of cubestable)(number (+ 1 ?num)) ))
)

(defrule only-speech-clear-upper-block
         (declare (salience -9200))
        (goal_only_speech (move ?block1))
        (stack ?top  $? ?block1 $?)
	=>
        (assert (goal_only_speech (move ?top)(on-top-of cubestable)))
)

(defrule only-speech-clear-lower-block
        (declare (salience -9100))
        (goal_only_speech (on-top-of ?block1))
        (stack ?top  $? ?block1 $?)
        =>
        (assert (goal_only_speech (move ?top)(on-top-of cubestable)))
)

(defrule only-speech-finish-plan
	(declare (salience -10000))
        (plan (name cubes_only_speech) (number ?num&:(neq ?num 0)))
        (not (plan (name cubes_only_speech) (number ?num1&:( > ?num1 ?num))))
        ?f <- (plan (name cubes_only_speech) (number 0))
        =>
	(retract ?f)
	(assert (finish-planner cubes_only_speech ?num))
)

(defrule only-speech-accomplish-plan
	(plan (name cubes_only_speech) (number ?num) (status accomplished))
	?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
	?f3 <- (item (type Objects) (name ?block1)) 
	?f4 <- (item (type Objects) (name ?block2)) 
	=>
	(retract ?f2)
	(modify ?f3 (status on-top-of ?block2))
	(modify ?f4 (status down-of ?block1))
)

(defrule only-speech-accomplish-plan-furniture
        (plan (name cubes_only_speech) (number ?num) (status accomplished))
        ?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
        ?f3 <- (item (type Objects) (name ?block1))
        ?f4 <- (item (type Furniture) (name ?block2)(status $?data))
        =>
        (retract ?f2)
        (modify ?f3 (status on-top-of ?block2))
        (modify ?f4 (status $?data ?block1))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
