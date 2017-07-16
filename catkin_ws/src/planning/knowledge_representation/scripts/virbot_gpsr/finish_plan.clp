;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;							;;
;;		finish_plan.clp				;;
;;		UNAM					;;
;;							;;
;;		Julio Cesar Cruz Estrada		;;
;;		04/Julio/2017				;;
;;							;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule finish_plan
	(declare (salience 100))
	?f <- (cmd_finish_plan 1)
	?f1 <- (name-scheduled ?plan 1 ?step)
	?f2 <- (state (name ?plan) (status active) (number ?n&:(neq ?n ?step)))
	?f3 <- (state (name ?plan) (status inactive) (number ?n2&:(eq ?n2 (- ?step 1))))
	?f4 <- (plan (name ?name) (number ?number) (status active))
	=>
	(printout t "I am going to finish the plan")
	(retract ?f)
	(modify ?f2 (status unaccomplished))
	(modify ?f3 (status active))
	(modify ?f4 (status unaccomplished))
)

(defrule finish_plan_final_navigation_active
	(declare (salience 100))
	?f <- (cmd_finish_plan 1)
	?f1 <- (name-scheduled ?plan 1 ?step)
	?f2 <- (state (name ?plan) (status active) (number ?n&: (eq ?n (- ?step 1))))
	=>
	(printout t "The last navigation in execution")
	(retract ?f)
)

(defrule finish_plan_final_task_active
	(declare (salience 100))
	?f <- (cmd_finish_plan 1)
	?f1 <- (name-scheduled ?plan 1 ?step)
	?f4 <- (state (name ?plan) (status active) (number ?step))
	=>	
	(printout t "The last task is in execution, So the plan continue")
	(retract ?f)
)



