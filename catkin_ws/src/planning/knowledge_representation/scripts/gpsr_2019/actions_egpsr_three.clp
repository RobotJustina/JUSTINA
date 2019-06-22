;;;;;;;;;;;;;;;;;
;;;;; EGPSR
;;;;;; Julio Cruz
;;;;; 7-06-2019
;;;;;;;;;;;;;


(defrule task_get_rpose_category
	?f <- (task ?plan get_rpose_object object ?place ?rpose ?category ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	(item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_rpose_obj ?place ?rpose ?category ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_pourin_object
	?f <- (task ?plan pourin_object ?pourable ?canpourin ?person ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?canpourin))
	;;;(item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Get something that can pourin ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_pourin)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task ppourin_obj ?pourable ?canpourin ?person ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_storage_object
	?f <- (task ?plan storage_object ?object ?loc ?storage ?sp_obj ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?object))
	=>
	(retract ?f)
	(printout t "Get something that can pourin ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_storage)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pstorage_obj ?object ?loc ?storage ?sp_obj ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_state_category
	?f <- (task ?plan storage_object ?category ?place ?step)
	?f1 <- (item (name finish_objetive))
	(item (type Category) (name ?category))
	(item (type Furniture) (name ?place))
	=>
	(retract ?f)
	(printout t "State what category ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_stated)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pset_obj ?category ?place ?step))
	(modify ?f1 (status nil))
	;(modify ?f2 (status nil))
)

(defrule task_storage_category
	?f <- (task ?plan storage_object ?storage ?sp_obj ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Storage Category ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_storage)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pstorage_category ?storage ?sp_obj ?step))
	(modify ?f1 (status nil))
)

(defrule task_get_object_description
	?f <- (task ?plan get_object_description object ?place ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Get object description ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_desc)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pobj_desc object ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_retrieve_object
	?f <- (task ?plan retrieve_object ?category ?place ?sp ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Get object description ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_retrieved)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pretrieve_obj ?category ?place ?sp ?step))
	(modify ?f1 (status nil))
)

(defrule task_interact_with_door
	?f <- (task ?plan interact_with_door ?door ?place ?action ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Open or close some door ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_close_or_open)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pinteract_with_door ?door ?place ?action ?step))
	(modify ?f1 (status nil))
)

(defrule task_set_tableware
	?f <- (task ?plan set_tableware ?tableware ?place ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Open or close some door ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_seted_tableware)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pset_tableware ?tableware ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_set_cutlery
	?f <- (task ?plan set_cutlery ?cutlery ?pos ?tableware ?place ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Open or close some door ")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_seted_cutlery)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pset_cutlery ?cutlery ?pos ?tableware ?place ?step))
	(modify ?f1 (status nil))
)

;;;;;;;;;;;;;;;;;;
;:;;;;;;;;;;;;;;;;;

(defrule plan_get_rpose_object
	?goal <- (objetive get_rpose_obj ?name ?place ?rpose ?category ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Relpos Object Task" crlf)
	(bind ?confirmation(str-cat "For this task I need your help, Do you want to help me, say justina yes or justina no"))
	(bind ?speech(str-cat "If you dont help me, I can not do the task"))
	(assert (plan (name ?name) (number 1)(actions confirmation ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions make_task ?name)(actions_num_params 3 3)(duration 6000)))
	;(assert (plan (name ?name) (number 3)(actions ask_for object ?place)(duration 6000)))
	;(assert (plan (name ?name) (number 4)(actions go_to object)(duration 6000)))
	;(assert (plan (name ?name) (number 5)(actions attend object)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-rpose-object ?place ?rpose ?category)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_task_neg ?name object grabed)(actions_num_params 5 5)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions speech-anything ?speech)(duration 6000)))
	;(assert (plan (name ?name) (number 5)(actions make_task ?name object finded)(actions_num_params 6 6)(duration 6000)))
	;(assert (plan (name ?name) (number 6)(actions move manipulator )(duration 6000))) 
	(assert (plan (name ?name) (number 6)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_canpourin_object
	?goal <- (objetive pourin_obj ?name ?pourable ?canpourin ?person ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Something that can pouring" crlf)
	(assert (plan (name ?name) (number 1) (actions pourin_object ?pourable ?canpourin ?person)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_pourin)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_storage_object
	?goal <- (objetive storage_obj ?name ?object ?loc ?storage ?sp_obj ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Something that can pouring" crlf)
	(assert (plan (name ?name) (number 1) (actions storage_object ?object ?loc ?storage ?sp_obj)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_storage)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_set_category
	?goal <- (objetive set_category ?name ?category ?loc ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Storage object" crlf)
	(assert (plan (name ?name) (number 1) (actions set_category_state ?category ?loc)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_stated)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_storage_category
	?goal <- (objetive storage_category ?name ?storage ?sp_obj ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Storage object" crlf)
	;(assert (plan (name ?name) (number 1) (actions get_category_state ?storage ?sp_obj)(duration 6000)))
	;(assert (plan (name ?name) (number 2) (actions storage_object)(duration 6000)))
	(assert (plan (name ?name) (number 1) (actions storage_object ?storage ?sp_obj)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_storage)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_get_object_description
	?goal <- (objetive obj_desc ?name ?obj ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Storage object" crlf)
	(assert (plan (name ?name) (number 1) (actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions obj_desc ?obj ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3) (actions update_status finish_objetive finaly_desc)(duration 6000)))
	(assert (finish-planner ?name 3))

)

(defrule plan_retrieve_object
	?goal <- (objetive retrieve_object ?name ?cat ?place ?sp ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Storage object" crlf)
	(assert (plan (name ?name) (number 1) (actions retrieve_object ?cat ?place ?sp)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_retrieved)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_interact_with_door
	?goal <- (objetive interact_with_door ?name ?door ?place ?action ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Storage object" crlf)
	(assert (plan (name ?name) (number 1) (actions interact_with_door ?door ?place ?action)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_close_or_open)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_set_tableware
	?goal <- (objetive set_tableware ?name ?tableware ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Set tableware" crlf)
	(assert (plan (name ?name) (number 1) (actions set_tableware ?tableware ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions update_status finish_objetive finaly_seted_tableware)(duration 6000)))
	(assert (finish-planner ?name 2))

)

(defrule plan_set_cutlery
	?goal <- (objetive set_cutlery ?name ?cutlery ?pos ?tableware ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Set tableware" crlf)
	(assert (plan (name ?name) (number 1) (actions make_task ?name ?tableware droped) (actions_num_params 2 2) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions set_cutlery ?cutlery ?pos ?tableware ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3) (actions update_status finish_objetive finaly_seted_cutlery)(duration 6000)))
	(assert (finish-planner ?name 3))
)
;;;;;;;;;;;;;;;;;;
(defrule exe_scheduled-get-rpose-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_rpose_obj ?place ?rpose ?category ?step)
	=>
	(retract ?f1)
	(assert (objetive get_rpose_obj task_get_rpose_obj ?place ?rpose ?category ?step))
)

(defrule exe_scheduled-pourin-object
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task ppourin_obj ?pourable ?canpourin ?person ?step)
	=>
	(retract ?f1)
	(assert (objetive pourin_obj task_pourin_obj ?pourable ?canpourin ?person ?step))
)

(defrule exe_scheduled-storage-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pstorage_obj ?object ?loc ?storage ?sp_obj ?step)
	=>
	(retract ?f1)
	(assert (objetive storage_obj task_storage_obj ?object ?loc ?storage ?sp_obj ?step))
)

(defrule exe_scheduled-state-category 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pset_obj ?category ?loc ?step)
	=>
	(retract ?f1)
	(assert (objetive set_category task_set_category ?category ?loc ?step))
)

(defrule exe_scheduled-storage-category 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pstorage_category ?storage ?sp_obj ?step)
	=>
	(retract ?f1)
	(assert (objetive storage_category task_storage_category ?storage ?sp_obj ?step))
)

(defrule exe_scheduled-get-object-description 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pobj_desc ?obj ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive obj_desc task_obj_desc ?obj ?place ?step))
)

(defrule exe_scheduled-retrieve-object  
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pretrieve_obj ?cat ?place ?sp ?step)
	=>
	(retract ?f1)
	(assert (objetive retrieve_object task_retrieve_object ?cat ?place ?sp ?step))
)

(defrule exe_scheduled-interact-with-door 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pinteract_with_door ?door ?place ?action ?step)
	=>
	(retract ?f1)
	(assert (objetive interact_with_door task_interact_with_door ?door ?place ?action ?step))
)

(defrule exe_scheduled-set-tableware 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pset_tableware ?tableware ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive set_tableware task_set_tableware ?tableware ?place ?step))
)

(defrule exe_scheduled-set-cutlery 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pset_cutlery ?cutlery ?pos ?tableware ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive set_cutlery task_set_cutlery ?cutlery ?pos ?tableware ?place ?step))
)
;;;;;;;;;;;;;;;;;;;
