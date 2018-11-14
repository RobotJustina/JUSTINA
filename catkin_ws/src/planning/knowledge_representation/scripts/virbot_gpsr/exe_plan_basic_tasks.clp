
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
	(item (name conf) (status ?st&:(or (eq ?st false) (eq ?st nil))))
	?f2 <- (finish-planner ?name ?n)
	=>
	(retract ?f2)
	(assert (f-plan ?name ?n ?ini ?end))
)

(defrule exe-plan-task-no-make-task_two
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?end&:(< ?ini ?end)))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	;;?f1 <- (state (name ?plan) (status active) (number ?n))
	(item (name conf) (status ?st&:(or (eq ?st false) (eq ?st nil))))
	(f-plan ?name ?n ?ini2 ?end2)
	=>
	(modify ?f1 (status accomplished))
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
)

(defrule exe-plan-task-no-make-last-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (+ ?ini 1))) (status inactive))
	?f3 <- (item (name conf) (status ?st&:(or(eq ?st false) (eq ?st nil))))
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
	?f4 <- (item (name conf) (status ?st&:(or (eq ?st false) (eq ?st nil))))
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
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-make-task-no-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task_neg ?name ?item ?status) (actions_num_params ?ini ?end))
	?f1 <- (item (name ?item) (status ?st&:(neq ?st ?status)))
	=>
	;(modify ?f1 (status nil))
	(modify ?f (status accomplished))
)

(defrule exe-plan-task-no-make-task-no-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task_neg ?name ?item ?status) (actions_num_params ?ini ?end))
	(item (name ?item) (status ?status))
	?f2 <- (finish-planner ?name ?n)
	=>
	(retract ?f2)
	(assert (f-plan ?name ?n ?ini ?end))
)

(defrule exe-plan-task-no-make-task_two-no-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task_neg ?name ?item ?status) (actions_num_params ?ini ?end&:(< ?ini ?end)))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	;;?f1 <- (state (name ?plan) (status active) (number ?n))
	(item (name ?item) (status ?status))
	(f-plan ?name ?n ?ini2 ?end2)
	=>
	(modify ?f1 (status accomplished))
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
)

(defrule exe-plan-task-no-make-last-task-no-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task_neg ?name ?item ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (+ ?ini 1))) (status inactive))
	?f3 <- (item (name ?item) (status ?status))
	?f4 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f4)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	(modify ?f2 (status active))
	;(modify ?f3 (status nil))
	(assert (finish-planner ?name ?n))
)

(defrule exe-plan-task-no-make-last-task-two-no-status 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task_neg ?name ?item ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini) (status inactive))
	?f4 <- (item (name ?item)(status ?status))
	?f3 <- (f-plan ?name ?n ?ini2 ?end2)
	=>
	(retract ?f3)
	(modify ?f1 (status accomplished))
	(modify ?f (status accomplished) (actions_num_params ?ini2 ?end2))
	;(modify ?f4 (status nil))
	(assert (finish-planner ?name ?n))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule stop_eegpsr
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?plan) (number 1) (status active))
	?f2 <- (plan (name ?name) (number ?num) (actions set_plan_status $?params)(status inactive))
	?f3 <- (plan (name ?name) (number ?ini&:(eq ?ini (+ 1 ?num))) (actions update_status ?item ?status) (status inactive))
	?f4 <- (plan (name ?name) (number ?num2) (actions $?actions)(status active))
	?f5 <- (finish-planner ?name ?ini)
	=>
	(retract ?f ?f5)
	(assert (accomplish ?plan ?name ?num2 (- ?num 1)))
	(assert (plan-f ?name ?ini))
	;(modify ?f4 (status accomplished))
)

(defrule accomplish_recursive
	?f <- (accomplish ?plan ?name ?ini ?end&:(neq ?ini ?end))
	?f1 <- (plan-f ?name ?i)
	?f2 <- (plan (name ?name) (number ?ini))
	=>
	(retract ?f)
	(assert (accomplish ?plan ?name (+ ?ini 1) ?end))
	(modify ?f2 (status accomplished))
)

(defrule accomplish_recursive_last
	?f <- (accomplish ?plan ?name ?ini ?ini)
	?f1 <- (plan-f ?name ?i)
	?f2 <- (plan (name ?name) (number ?ini))
	?f3 <- (plan (name ?name) (number ?num&:(eq ?num (+ 1 ?ini))))
	=>
	(retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status active))
	(assert (finish-planner ?name ?i))
)

(defrule no_stop_eegpsr_last_two_task_active
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?name) (number 1) (status active))
	?f3 <- (finish-planner ?name ?end)
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (- ?end 1))) (actions set_plan_status $?params)(status active))
	=>
	(retract ?f)
)

(defrule no_stop_eegpsr_last_task_active
        (declare (salience 1000))
	?f <- (cmd_stop_eegpsr 1)
	?f1 <- (state (name ?name) (number 1) (status active))
	?f2 <- (plan (name ?name) (number ?num) (actions update_status ?item ?status) (status active))
	?f5 <- (finish-planner ?name ?num)
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;		dummy tasks		;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;
;;; dumy task os set_plan_status

(defrule exe-plan-set-plan-status-dummy 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_plan_status ?name dummy))
	;?f1 <- (finish-planner ?name ?n)
	=>
	;(retract ?f1)
	;(assert (set_plan_status ?name ?n))
	(modify ?f (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; rule for update location coords
(defrule exe-update-location-coords
	(plan (name ?name) (number ?num-pln) (status active) (actions update_location_coords ?location) (duration ?t))
	=>
	(bind ?command (str-cat "" ?location ""))
	(assert (send-blackboard ACT-PLN cmd_update_loc_coords ?command ?t 4))
)

(defrule exe-plan-updated-location-coords 
	?f <- (received ?sender command cmd_update_loc_coords ?loc 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions update_location_coords ?loc))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-no-updated-location-coords 
	?f <- (received ?sender command cmd_update_loc_coords ?loc 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions update_location_coords ?loc))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
