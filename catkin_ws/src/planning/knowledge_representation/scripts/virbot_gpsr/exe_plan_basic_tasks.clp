
;;;;;;;;;;;;; confirmation

(defrule exe-plan-task-confirmation
	(plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf) (duration ?t))
	=>
	(bind ?command (str-cat "" ?conf ""))
	(assert (send-blackboard ACT-PLN cmd_task_conf ?command ?t 4))
)

(defrule exe-plan-task-confirmated
	?f <- (received ?sender command cmd_task_conf conf 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf))
	?f2 <- (item (name conf))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status true))
)

(defrule exe-plan-task-no-confirmated
	?f <- (received ?sender command cmd_task_conf conf 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf))
	?f2 <- (item (name conf))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status false))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-task-make-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?end))
	?f1 <- (item (name conf) (status true))
	=>
	(modify ?f1 (status nil))
	(modify ?f (status accomplished))
)

(defrule exe-plan-task-no-make-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?end))
	(item (name conf) (status false))
	?f2 <- (finish-planner ?name ?n)
	=>
	(retract ?f2)
	(assert (f-plan ?name ?n ?ini ?end))
)

(defrule exe-plan-task-no-make-task_two
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?end&:(< ?ini ?end)))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	;;?f1 <- (state (name ?plan) (status active) (number ?n))
	(item (name conf) (status false))
	(f-plan ?name ?n ?ini2 ?end2)
	=>
	(modify ?f1 (status accomplished))
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
)

(defrule exe-plan-task-no-make-last-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (+ ?ini 1))) (status inactive))
	?f3 <- (item (name conf) (status false))
	?f4 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f4)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	(modify ?f2 (status active))
	(modify ?f3 (status nil))
	(assert (finish-planner ?name ?n))
)

(defrule exe-plan-task-no-make-last-task-two
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f4 <- (item (name conf) (status false))
	?f3 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f3)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	(modify ?f4 (status nil))
	(assert (finish-planner ?name ?n))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-make-task-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?item ?status) (actions_num_params ?ini ?end))
	?f1 <- (item (name ?item) (status ?status))
	=>
	;(modify ?f1 (status nil))
	(modify ?f (status accomplished))
)

(defrule exe-plan-task-no-make-task-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?item ?status) (actions_num_params ?ini ?end))
	(item (name ?item) (status ?st&:(neq ?status ?st)))
	?f2 <- (finish-planner ?name ?n)
	=>
	(retract ?f2)
	(assert (f-plan ?name ?n ?ini ?end))
)

(defrule exe-plan-task-no-make-task_two-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?item ?status) (actions_num_params ?ini ?end&:(< ?ini ?end)))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	;;?f1 <- (state (name ?plan) (status active) (number ?n))
	(item (name ?item) (status ?st&:(neq ?st ?status)))
	(f-plan ?name ?n ?ini2 ?end2)
	=>
	(modify ?f1 (status accomplished))
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
)

(defrule exe-plan-task-no-make-last-task-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?item ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (+ ?ini 1))) (status inactive))
	?f3 <- (item (name ?item) (status ?st&:(neq ?st ?status)))
	?f4 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f4)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	(modify ?f2 (status active))
	(modify ?f3 (status nil))
	(assert (finish-planner ?name ?n))
)

(defrule exe-plan-task-no-make-last-task-two-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?item ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f4 <- (item (name ?item)(status ?st&:(neq ?st ?status)))
	?f3 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f3)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	(modify ?f4 (status nil))
	(assert (finish-planner ?name ?n))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule stop_eegpsr
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?plan) (number 1) (status active))
	?f2 <- (plan (name ?name) (number ?num) (actions set_plan_status ?name)(status inactive))
	?f3 <- (plan (name ?name) (number ?num1) (actions update_status ?item ?status) (status inactive))
	?f4 <- (plan (name ?name) (number ?num2) (actions $?actions)(status active))
	=>
	(retract ?f)
	(modify ?f2 (status active))
	(modify ?f4 (status accomplished))
)

(defrule no_stop_eegpsr_last_two_task_active
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?name) (number 1) (status active))
	?f2 <- (plan (name ?name) (number ?num) (actions set_plan_status ?name)(status active))
	=>
	(retract ?f)
)

(defrule no_stop_eegpsr_last_task_active
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?name) (number 1) (status active))
	?f2 <- (plan (name ?name) (number ?num) (actions update_status ?item ?status) (status active))
	=>
	(retract ?f)
)

(defrule no_stop_eegpsr
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	(not (state (name ?name) (number 1) (status active)))
	=>
	(retract ?f)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
