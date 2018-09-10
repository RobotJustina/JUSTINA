;********************************************************
;*                                                      *
;*      exe_plan_iros.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      6/09/2018                      *
;*                                                      *
;********************************************************

(defrule exe_plan_set_param_in_plan_init
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_param_in_plan ?obj) (actions_num_params ?iniPlan ?endPlan ?iniParamPila ?endParamPila))
	?f1 <- (init_pile set_params)
	=>
	(retract ?f1)
	(assert (set_param_limits ?iniPlan ?endPlan))
)

(defrule exe_plan_set_param_in_plan
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_param_in_plan ?obj) (actions_num_params ?iniPlan ?endPlan&:(neq ?iniPlan ?endPlan) ?iniParamPila ?endParamPila))
	?f1 <- (item (type Pile) (name ?topPile)(image ?obj)(zone ?zone) (num ?endParamPila))
	?f2 <- (plan (name ?name) (number ?iniPlan)(actions ?action $?params ?dummy_param) (statusTwo plan_active))
	?f3 <- (set_param_limits ?ini ?end)
	=>
	(modify ?f (actions_num_params (+ 1 ?iniPlan) ?endPlan ?iniParamPila ?endParamPila))
	(modify ?f2 (actions ?action $?params ?zone))
)

(defrule exe_plan_set_param_finish_plan
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_param_in_plan ?obj) (actions_num_params ?iniPlan ?iniPlan ?iniParamPile ?endParamPile))
	?f1 <- (item (type Pile) (name ?topPile) (image ?obj) (zone ?zone) (num ?endParamPile))
	?f2 <- (plan (name ?name) (number ?iniPlan)(actions ?action $?params ?dummy_param)(statusTwo plan_active))
	?f3 <- (set_param_limits ?ini ?end)
	=>
	(retract ?f3)
	(assert (init_pile set_params))
	(modify ?f (status accomplished)(actions_num_params ?ini ?end ?iniParamPile (- ?endParamPile 1)))
	(modify ?f2 (actions ?action $?params ?zone))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
