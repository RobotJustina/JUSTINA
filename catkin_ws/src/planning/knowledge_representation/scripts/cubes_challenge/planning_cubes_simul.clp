;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;						;;;
;;;		University of Mexico		;;;
;;;		Julio Cesar Cruz Estrada	;;;
;;;		20/12/2017			;;;
;;;						;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftemplate goal_simul (slot move)(slot on-top-of))
(deftemplate attempt_simul (slot move)(slot on-top-of)(slot number))
(defglobal ?*simul_plan_time* = 30000)


(deffacts initial-simul-state
	(plan (name cubes_simul) (number 0)(duration 0))

)
;;;;; reglas para crear y ordenar el plan simulado

(defrule pop-first-stack
	?f2 <- (pop first_stack ?block2)
	?f <- (pile (name simul) (first_stack $?rest ?block&:(neq ?block nil)) (number ?num))
	=>
	(retract ?f2)
	(assert (move ?block ?block2 first_stack (+ 1 ?num)))
	(assert (pop first_stack ?block))
	(modify ?f (first_stack $?rest)(number (+ 1 ?num)))
)

(defrule pop-empty-first-stack
	?f2 <- (pop first_stack ?block2)
	?f <- (pile (name simul) (first_stack ))
	=>
	(retract ?f2)
	(assert (start cubes_simul first_stack))
)

(defrule pop-second-stack
	?f2 <- (pop second_stack ?block2)
	?f <- (pile (name simul) (second_stack $?rest ?block&:(neq ?block nil))(number ?num))
	=>
	(retract ?f2)
	(assert (move ?block ?block2 second_stack (+ 1 ?num)))
	(assert (pop second_stack ?block))
	(modify ?f (second_stack $?rest)(number (+ 1 ?num)))
)

(defrule pop-empty-second-stack
	?f2 <- (pop second_stack ?block2)
	?f <- (pile (name simul) (second_stack ))
	=>
	(retract ?f2)
	(assert (start cubes_simul second_stack))
)

;;;;;;;;;;;;;;;;; reglas para ir activando el plan simulado
(defrule init-simul
	;?f <- (start cubes_simul first_stack)
	;?f1 <- (start cubes_simul second_stack)
	?f <- (start simul)
	;?f2 <- (move ?block1 ?block2 ?stack 1)
	?f2 <- (move ?block1 ?block2 1)
	=>
	(printout t "Inicia la simulacion" crlf)
	(retract ?f)
	(assert (goal_simul (move ?block1) (on-top-of ?block2)))
)

(defrule next-plan-simul
	?f <- (item (name ?block1) (attributes on-top ?block2))
	?f1 <- (move ?block1 ?block2 ?num)
	;?f2 <- (pile (name simul)(number ?numsim&:(< ?num ?numsim)))
	(simul_moves ?numsim&:(< ?num ?numsim))
	?f3 <- (move ?block3 ?block4 ?n&:(eq ?n (+ 1 ?num)))
	=>
	(retract ?f1)
	(modify ?f (attributes nil))
	(assert (goal_simul (move ?block3) (on-top-of ?block4)))
	(printout t "siguiente plan es: " ( + 1 ?num) " test: " ?n ""  crlf)
)

(defrule last-plan-simul
	?f <- (item (name ?block1) (attributes on-top ?block2))
	?f1 <- (move ?block1 ?block2 ?num)
	;?f2 <- (pile (name simul) (number ?numsim&:(eq ?num ?numsim)))
	?f2 <- (simul_moves ?numsim&:(eq ?num ?numsim))
	?f3 <- (pile (name original))
	?f4 <- (item (name stack))
	?f5 <- (plan (name ?name) (number ?num-pln) (status active) (actions make-backtracking) (duration ?t))
	=>
	(printout t "Se finaliza ultima tarea del plan" crlf)
	(retract ?f1 ?f2)
	(modify ?f (attributes nil))
	(modify ?f4 (status review))
	;(modify ?f2 (status nil) (number 0))
	(assert (simul_moves 0))
	(modify ?f3 (status nil))
	(modify ?f5 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; simulacion de acciones ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule simul-dont-move-directly
        (declare (salience 1000))
	?goal <- (goal_simul (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
	?stack-1 <- (stack $?top ?block1 ?block2 $?bottom)
	(plan (name cubes_simul) (number ?num))
	(not (plan (name cubes_simul) (number ?num1&:( > ?num1 ?num))))
	=>
	(retract ?goal)
	(printout t "" ?block1 " remains on top of " ?block2 "." crlf)
	(bind ?speech (str-cat "" ?block1 " remains on top of " ?block2 ""))
	(assert (plan (name cubes_simul) (number 1)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_simul) (number 2)(actions pile_simul ?block1 ?block2 upgrade_state)(duration 6000)))
	(assert (finish-planner cubes_simul 2))
)

(defrule simul-dont-move-to-table
	(declare (salience 1000))
	?goal <- (goal_simul (move ?block1) (on-top-of cubestable))
	?stack-1 <- (stack $?top ?block1)
	(plan (name cubes_simul) (number ?num))
	(not (plan (name cubes_simul) (number ?num1&:(> ?num1 ?num))))
	
	=>
	(retract ?goal)
	(printout t "" ?block1 " remains on top of the table." crlf )
	(bind ?speech (str-cat "" ?block1 " remains on top of the table"))
	(assert (plan (name cubes_simul) (number 1) (actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_simul) (number 2) (actions pile_simul ?block1 cubestable upgrade_state)(duration 6000)))
	(assert (finish-planner cubes_simul 2))
)

(defrule simul-move-directly
        ?goal <- (goal_simul (move ?block1) (on-top-of ?block2&:(neq ?block2 cubestable)))
        ?stack-1 <- (stack ?block1 $?rest1)
        ?stack-2 <- (stack ?block2 $?rest2)
	(plan (name cubes_simul) (number ?num))
	(not (plan (name cubes_simul) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        (printout t ?block1 " was moved on top of " ?block2 "." crlf)
	(bind ?speech (str-cat "" ?block1 " was moved on top of " ?block2 ""))
	(assert (plan (name cubes_simul) (number 1)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_simul) (number 2)(actions enable_arm ?block1)(duration 6000)))
        (assert (plan (name cubes_simul) (number 3)(actions move_simul manipulator ?block1 )(duration 6000)))
        (assert (plan (name cubes_simul) (number 4 )(actions grab manipulator ?block1 )(duration 6000)))
        (assert (plan (name cubes_simul) (number 5 )(actions place_block_simul ?block1 ?block2)(duration 6000)))
	(assert (plan (name cubes_simul) (number 6)(actions pile_simul ?block1 ?block2)(duration 6000)))
	(assert (finish-planner cubes_simul 6))
	;(assert (attempt (move ?block1) (on-top-of ?block2)(number (+ 1 ?num) )))
)

(defrule simul-move-to-table
        ?goal <- (goal_simul (move ?block1) (on-top-of cubestable))
        ?stack-1 <- (stack ?block1 $?rest)
	(plan (name cubes_simul) (number ?num))
        (not (plan (name cubes_simul) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        (printout t ?block1 " was moved on top of the table." crlf)
	(bind ?speech (str-cat "" ?block1 " was moved on top of the table"))
	(assert (plan (name cubes_simul) (number 1)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name cubes_simul) (number 2)(actions enable_arm ?block1)(duration 6000)))
        (assert (plan (name cubes_simul) (number 3)(actions move_simul manipulator ?block1 )(duration 6000)))
        (assert (plan (name cubes_simul) (number 4)(actions grab manipulator ?block1 )(duration 6000)) )
        (assert (plan (name cubes_simul) (number 5)(actions place_block_simul ?block1 ?block1)(duration 6000)) )
	(assert (plan (name cubes_simul) (number 6)(actions pile_simul ?block1)(duration 6000)))
	(assert (finish-planner cubes_simul 6))
	;(assert (attempt (move ?block1) (on-top-of cubestable)(number (+ 1 ?num)) ))
)

(defrule simul-clear-upper-block
         (declare (salience -9200))
        (goal_simul (move ?block1))
        (stack ?top  $? ?block1 $?)
	=>
        (assert (goal_simul (move ?top)(on-top-of cubestable)))
)

(defrule simul-clear-lower-block
        (declare (salience -9100))
        (goal_simul (on-top-of ?block1))
        (stack ?top  $? ?block1 $?)
        =>
        (assert (goal_simul (move ?top)(on-top-of cubestable)))
)

(defrule simul-finish-plan
	(declare (salience -10000))
        (plan (name cubes_simul) (number ?num&:(neq ?num 0)))
        (not (plan (name cubes_simul) (number ?num1&:( > ?num1 ?num))))
        ?f <- (plan (name cubes_simul) (number 0))
        =>
	(retract ?f)
	(assert (finish-planner cubes_simul ?num))
)

(defrule simul-accomplish-plan
	(plan (name cubes_simul) (number ?num) (status accomplished))
	?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
	?f3 <- (item (type Objects) (name ?block1)) 
	?f4 <- (item (type Objects) (name ?block2)) 
	=>
	(retract ?f2)
	(modify ?f3 (status on-top-of ?block2))
	(modify ?f4 (status down-of ?block1))
)

(defrule simul-accomplish-plan-furniture
        (plan (name cubes_simul) (number ?num) (status accomplished))
        ?f2 <- (attempt (move ?block1) (on-top-of ?block2) (number ?num))
        ?f3 <- (item (type Objects) (name ?block1))
        ?f4 <- (item (type Furniture) (name ?block2)(status $?data))
        =>
        (retract ?f2)
        (modify ?f3 (status on-top-of ?block2))
        (modify ?f4 (status $?data ?block1))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
