;;;;;;;

;find_person_in_room			params 1	Ready
;wait_for_user_instruction
;update_object_location			params 2	Ready (crackers bathroom)
;get_object				params 1	Ready
;put_object_in_location			params 2	
;save_position				params 1	(current_loc) 
;deliver_in_position			params 2	exemple (pringles current_loc) 
;handover_object			params 1	Ready



(defrule task_update_object_location
	?f <- (task ?plan update_object_location ?param1 ?param2 ?step)
	?f1 <- (item (name ?param2))
	;?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Object location task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?param2)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	;(modify ?f2 (zone frontexit))
		
)

(defrule task_put_object_in_location
	?f <- (task ?plan put_object_in_location ?param1 ?param2 ?step)
	?f1 <- (item (name ?param1))
	=>
	(retract ?f)
	(printout t "Put object in location task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pobjloc) (actor robot)(obj robot)(from ?param1)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	
		
)

(defrule task_deliver_in_position
	?f <- (task ?plan deliver_in_position ?param1 ?param2 ?step)
	?f1 <- (item (name ?patam1))
	;?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Deliver in position" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pobjloc) (actor robot)(obj robot)(from ?param1)(to ?param2)(name-scheduled ?plan)(state-number ?step)))
	;(assert (condition (conditional if) (arguments robot zone kitchen)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	;(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to kitchen)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	;(modify ?f2 (zone frontexit))
	
		
)



(defrule task_find_person_in_room
	?f <- (task ?plan find_person_in_room ?param1 ?step)
	?f1 <- (item (name ?param1))
	?f2 <- (item (name person))
	=>
	(retract ?f)
	(printout t "Find person in room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments person status went)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfindperson) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)


(defrule task_find_specific_person_in_room
	?f <- (task ?plan find_person_in_room ?person ?place ?step)
	?f1 <- (item (name ?place))
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Find Specific person in room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?person status went)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfindspcperson) (actor robot)(obj robot)(from ?person)(to ?place)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_object
	?f <- (task ?plan get_object ?param1 ?place ?step)
	?f1 <- (item (name ?param1)(type Objects))
	=>
	(retract ?f)
	(printout t "Get object" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pgetobj) (actor robot)(obj robot)(from ?place)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;;;;;;
	(modify ?f1 (status nil))	
)

(defrule task_get_object_man
	?f <- (task ?plan get_object man ?place ?step)
	?f1 <- (item (name man)(type Person))
	=>
	(retract ?f)
	(printout t "Get object MAN" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments man status followed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pgetobjman) (actor robot)(obj robot)(from ?place)(to man)(name-scheduled ?plan)(state-number ?step)))
	;;;;;;;;;;;
	(modify ?f1 (status nil))	
)

(defrule task_save_position
	?f <- (task ?plan save_position ?param1 ?step)
	?f1 <- (item (name ?param1))
	?f2 <- (item (name robot) (zone ?zone))
	=>
	(retract ?f)
	(printout t "Save position" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments robot zone ?param1)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	;(modify ?f1 (status nil))
	(modify ?f2 (zone frontexit))
)


(defrule task_handover_object
	?f <- (task ?plan handover_object ?param1 ?step)
	?f1 <- (item (name ?param1))
	=>
	(retract ?f)
	(printout t "Handover object" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd phandover) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
)

(defrule task_handover_object_person
        ?f <- (task ?plan handover_object ?param1 ?person ?step)
        ?f1 <- (item (name ?param1))
        (item (name ?person))
        =>
        (retract ?f)
        (printout t "Handover object" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?param1 status droped)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd phandover) (actor robot)(obj robot)(from frontexit)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
        ;;;;;test reiniciar status del parametro
        (modify ?f1 (status nil) (possession ?person))
)

(defrule task_wait_for_user_instruction
	?f <- (task ?plan wait_for_user_instruction ?question_task ?step)
	?f1 <- (item (name question))
	?f2 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Wait for user instruction" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments question status ask)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pquestion) (actor robot)(obj robot)(from ?question_task)(to kitchen)(name-scheduled ?plan)(state-number ?step)))
	
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
	(modify ?f2 (zone frontexit))

	

	
)

(defrule task_put_on_top
	?f <- (task ?plan put_on_top ?block1 ?block2 ?step)
	?f1 <- (item (name ?block1))
	?f2 <- (item (name ?block2))
	?f3 <- (item (name robot))
	=>
	(retract ?f)
	(printout t "Put block1 on top Block2" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?block1 status on-top) (true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (condition-block ?block1 ?block2))
	(assert (cd-task (cd pOnTop) (actor robot)(obj robot)(from ?block1)(to ?block2)(name-scheduled ?plan)(state-number ?step)))

	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
	;(modify ?f3 (zone frontexit))

)

;;;;; task para el cubes chalenge: (revision del estado de las pilas, y simulación de acciones)
(defrule task_review_pile_state
	?f <- (task ?plan stack_state person ?step)
	?f1 <- (item (name robot))
	?f2 <- (item (name stack))
	=>
	(retract ?f)
	(printout t "Review pile state" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments stack status review) (true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pPileState)(actor robot)(obj robot)(from frontexit)(to kitchen)(name-scheduled ?plan)(state-number ?step)))

	(modify ?f2 (status nil))
)

(defrule task_speech_generator
        ?f <- (task ?plan speech_generator ?name ?step)
        ?f1<- (item (type Speech)(name ?name) (image ?spg))
        =>
        (retract ?f)
        (printout t "Task for Speech generator" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?name status said)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pspg) (actor robot)(obj robot)(from exitdoor)(to ?spg)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_explain_cubes_plan
	?f <- (task ?plan explain_cubes_plan ?block1 ?block2 ?step)
	?f1 <- (item (name stack_exp))
	=>
	(retract ?f)
	(printout t "Task for explain the cubes plan" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments stack_exp status explained)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd p_explain_cplan)(actor robot)(obj robot)(from ?block1)(to ?block2)(name-scheduled ?plan)(state-number ?step)))
	(modify ?f1 (status nil))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule plan_ask_for
        ?goal <- (objetive get_obj ?name ?param ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions ask_for ?param ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?param)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?param)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-object ?param)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions move manipulator ?param)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions grab manipulator ?param)(duration 6000)))
	;(assert (into (name ?name)(number ?step)(next (+ ?step 1))(plan 6)))
	(assert (finish-planner ?name 6))
)


(defrule plan_find_person
        ?goal <- (objetive find_person ?name ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?param)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-object person)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_person person)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_handover_obj
        ?goal <- (objetive handover_obj ?name ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	;(assert (plan (name ?name) (number 1)(actions move manipulator person)(duration 6000)))
	(assert (plan (name ?name) (number 1)(actions drop person ?param)(duration 6000)))
	(assert (finish-planner ?name 1))
)

(defrule plan_put_obj_in_loc
        ?goal <- (objetive put_obj_loc ?name ?param1 ?param2 ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?param2)(duration 6000)))
	;(assert (plan (name ?name) (number 2)(actions ask_for ?param1)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions go_to ?param1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions attend ?param1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions move manipulator ?param1)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions drop object ?param1)(duration 6000)))
	(assert (finish-planner ?name 4))
)

(defrule plan_answer_question
        ?goal <- (objetive answer_question ?name ?question_task ?param2 ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Answer question Task" crlf)
	(assert (plan (name ?name) (number 1)(actions answer_question question ?question_task)(duration 6000)))
	(assert (finish-planner ?name 1))
)


(defrule plan_get_obj_man
        ?goal <- (objetive get_obj_man ?name ?place ?param ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions find-object-man ?param ?place)(duration 6000)))
	(assert (finish-planner ?name 1))
)


(defrule plan_find_person_spc
        ?goal <- (objetive find_spc_person ?name ?person ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-object specific ?person)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_put_on_top
	?goal <- (objetive put_on_top ?name ?block1 ?block2 ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Move Blok")
	(bind ?speech(str-cat "I am ready for another petition"))
	(assert (plan (name ?name) (number 1) (actions enable_simul False) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions put_on_top ?block1 ?block2) (duration 6000)))
	;(assert (plan (name ?name) (number 3) (actions update_stack ?block1) (duration 6000)))
	(assert (plan (name ?name) (number 3) (actions speech-anything ?speech) (duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_review_pile_state
	?goal <- (objetive pile_state ?name ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Review Pile State" crlf)
	(assert (plan (name ?name) (number 1) (actions review) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions speech_dynamic_stack_state) (duration 6000)))
	(assert (plan (name ?name) (number 3) (actions enable_simul True) (duration 6000)))
	(assert (plan (name ?name) (number 4) (actions make-backtracking) (duration 6000)))
	(assert (finish-planner ?name 4))
)

(defrule plan_speech_generator
        ?goal <- (objetive speech_generator ?name ?spg ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan speech generator" crlf)
        (assert (plan (name ?name) (number 1)(actions speech_generator)(duration 6000)))
        (assert (finish-planner ?name 1))
)

(defrule plan_explain_cubes_plan
	?goal <- (objetive explain_cubes_plan ?name ?block1 ?block2 ?step)
	=>
	(retract ?goal)
	(printout t "Prueba nuevo PLan explain cubes plan" crlf)
	(bind ?speech(str-cat "Ok I am going to explain the plan"))
	(bind ?speech1(str-cat "Now i start to execute the command"))
	(bind ?confirmation(str-cat "Explain_the_plan"))
	(assert (plan (name ?name) (number 1)(actions backup_stacks first)))
	(assert (plan (name ?name) (number 2)(actions confirmation ?confirmation)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name stack_exp explained) (actions_num_params 4 14)))
	(assert (plan (name ?name) (number 4)(actions speech-anything ?speech)))
	(assert (plan (name ?name) (number 5)(actions backup_cubes blue_block)))
	(assert (plan (name ?name) (number 6)(actions backup_cubes red_block)))
	(assert (plan (name ?name) (number 7)(actions backup_cubes green_block)))
	(assert (plan (name ?name) (number 8)(actions enable_simul True)))
	;(assert (plan (name ?name) (number 9)(actions go_to_place table)))
	(assert (plan (name ?name) (number 9)(actions put_on_top_only_speech ?block1 ?block2)))
	;(assert (plan (name ?name) (number 10)(actions go_to_place dining_room)))
	(assert (plan (name ?name) (number 10)(actions enable_simul False)))
	(assert (plan (name ?name) (number 11)(actions update_status original nil)))
	(assert (plan (name ?name) (number 12)(actions restore_cubes blue_block)))
	(assert (plan (name ?name) (number 13)(actions restore_cubes red_block)))
	(assert (plan (name ?name) (number 14)(actions restore_cubes green_block)))
	;(assert (plan (name ?name) (number 15)(actions reset_cube_pos blue_block)))
	;(assert (plan (name ?name) (number 16)(actions reset_cube_pos red_block)))
	;(assert (plan (name ?name) (number 17)(actions reset_cube_pos green_block)))
	(assert (plan (name ?name) (number 15)(actions restore_stacks_dynamic delate)))
	(assert (plan (name ?name) (number 16)(actions speech-anything ?speech1)))
	(assert (plan (name ?name) (number 17)(actions update_status stack_exp explained)))
	(assert (finish-planner ?name 17))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_scheduled-get
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgetobj) (actor ?robot)(obj ?robot)(from ?place)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive get_obj task_get ?param1 ?place ?step))
)

(defrule exe_scheduled-find
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfindperson) (actor ?robot)(obj ?robot)(from ?from)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_person task_find ?param1 ?step))
)

(defrule exe_scheduled-handover
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd phandover) (actor ?robot)(obj ?robot)(from ?from)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive handover_obj task_handover ?param1 ?step))
)


(defrule exe_scheduled-put-object-in-location
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pobjloc) (actor ?robot)(obj ?robot)(from ?param1)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive put_obj_loc task_pobjloc ?param1 ?param2 ?step))
)

(defrule exe_wait_for_user_instruction
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pquestion) (actor ?robot)(obj ?robot)(from ?question_task)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive answer_question task_aquestion ?question_task ?param2 ?step))
)


(defrule exe_get_object_man
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgetobjman) (actor ?robot)(obj ?robot)(from ?place)(to ?param2)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive get_obj_man task_get_man ?place ?param2 ?step))
)

(defrule exe_scheduled-find-specific
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfindspcperson) (actor ?robot)(obj ?robot)(from ?person)(to ?place)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_spc_person task_find_spc ?person ?place ?step))
)

(defrule exe_move_block_on-top
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (cd-task (cd pOnTop) (actor ?robot)(obj ?robot) (from ?block1) (to ?block2) (name-scheduled ?name) (state-number ?step))
	=>
	(retract ?f1)
	(assert (objetive put_on_top task_put_on_top ?block1 ?block2 ?step))
        ;(assert(goal (move ?block1)(on-top-of ?block2)))
)


(defrule exe_review_pile_state
	(state (name ?name) (number ?step)(status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (cd-task (cd pPileState)(actor ?robot)(obj ?robot)(from ?person)(to ?place)(name-scheduled ?name)(state-number ?step))
	=>
	(retract ?f1)
	(assert (objetive pile_state task_pile_state ?step))
)


(defrule exe_speech-generator
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pspg) (actor ?robot)(obj ?robot)(from ?place)(to ?spg)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive speech_generator task_speech_generator ?spg ?step))
)

(defrule exe_plan_explain_cubes_plan
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (cd-task (cd p_explain_cplan) (actor ?robot) (obj ?robot)(from ?block1)(to ?block2)(name-scheduled ?name)(state-number ?step))
	=>
	(retract ?f1)
	(assert (objetive explain_cubes_plan task_explain_cubes_plan ?block1 ?block2 ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



