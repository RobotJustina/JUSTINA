;********************************************************
;*                                                      *
;*      actions_three.clp                               *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      28/03/2019                      *
;*                                                      *
;********************************************************

(defrule task_get_object_room
	?f <- (task ?plan get_object ?obj ?room ?step)
	?f1 <- (item (name ?obj)(type Objects))
        (item (type Room) (name ?room))
	?f2 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Try to find the object inside the room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pgobjroom ?obj ?room ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_bag
	?f <- (task ?plan get_bag ?luggage ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Get luggage and try to take it to some taxi or car" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_bag ?step))
	(modify ?f1 (status nil))

)

(defrule task_follow_to_taxi
	?f <- (task ?plan follow_to_taxi man ?place ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Task follow to taxi, uber, cab" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_followed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfollow_to_taxi ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_get_abspos_object
	?f <- (task ?plan get_object ?object ?place ?abspos ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?object))
	=>
	(retract ?f)
	(printout t "Get abspos object" crlf)
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_abspos_obj ?object ?place ?abspos ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_relpos_object
	?f <- (task ?plan get_object object ?place ?relpos ?object ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	(item (type Objects)(name ?object))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_relpos_obj object ?place ?relpos ?object ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_oprop_category
	?f <- (task ?plan get_object object ?place ?oprop ?category ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	(item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_oprop_obj ?place ?oprop ?category ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_oprop_object
	?f <- (task ?plan get_object object ?place ?oprop nil ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_oprop_obj ?place ?oprop nil ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_find_three_oprop_object
	?f <- (task ?plan find_prop_object ?oprop nil three ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfind_three_oprop_obj ?oprop nil three ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_find_three_oprop_category
	?f <- (task ?plan find_prop_object ?oprop ?category three ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name object))
	(item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Get relpos object")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfind_three_oprop_obj ?oprop ?category three ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)


(defrule task_find_three_oprop_category
	?f <- (task ?plan find_category_room ?category ?place three ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (type Category) (name ?category))
	=>
	(retract ?f)
	(printout t "Find three objects of some category in some room")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?category status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfind_three_cat ?category ?place three ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_find_person_in_door
	?f <- (task ?plan find_person_in_door ?person ?door ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?person))
	?f3 <- (item (name ?door) (image ?location))
	?f4 <- (item (name ?location))
	=>
	(retract ?f)
	(printout t "Find a person in the door")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_went)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pfind_person_in_door ?person ?location ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status current_person))
	(modify ?f4 (status nil))
)

(defrule task_introduce_to_people
	?f <- (task ?plan introduce_person ?php ?place ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?place))
	?f3 <- (item (name ?person) (status current_person))
	=>
	(retract ?f)
	(printout t "Introduce person to people")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_introduced)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pintroduce_person people ?person ?php ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
	(modify ?f3 (status nil))
)

(defrule task_introduce_to_person
	?f <- (task ?plan introduce_person ?person1 ?place ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?place))
	?f3 <- (item (name ?person2) (status current_person))
	?f4 <- (item (name ?person1))
	=>
	(retract ?f)
	(printout t "Introduce person to people")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_introduced)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pintroduce_person person ?person2 ?person1 ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
	(modify ?f3 (status nil))
)

(defrule task_make_a_question
	?f <- (task ?plan make_question ?question ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?person) (status current_person))
	=>
	(retract ?f)
	(printout t "Justina Make a question")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_asked)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pmake_question ?person ?question ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_guide_to_taxi
	?f <- (task ?plan guide_to_taxi ?person ?question ?step)
	?f1 <- (item (name finish_objetive))
	?f2 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Guide to taxi")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_guided)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pguide_to_taxi ?person ?question ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_clean_up
	?f <- (task ?plan clean_up ?room ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Clean Up")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_cleaned)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pclean_up ?room ?step))
	(modify ?f1 (status nil))
)

(defrule task_take_out_the_garbage
	?f <- (task ?plan take_out_garbage ?garbage ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Take out the garbage")
	(assert (state (name ?plan)(number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_taked_out)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task ptake_out_garbage ?garbage ?step))
	(modify ?f1 (status nil))
)

(defrule task_find_category_incomplete
	?f <- (task ?plan find_category_room ?category ?step)
	?f1 <- (item (name ?category)(type Category))
	?f2 <- (item (type Room) (name ?room) (image current_place))
	=>
	(retract ?f)
	(printout t "Find category in room")
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?category status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pfindobj_room) (actor robot)(obj robot)(from ?room)(to ?category)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
	(modify ?f2 (image nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;  SPLIT IN SUBTAREAS

(defrule plan_get_object_room
        ?goal <- (objetive gobjroom ?name ?obj ?room ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?obj ?room)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions review_room ?obj ?room)(actions_num_params 3 5 1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name ?obj finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions move manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 100))
)

(defrule plan_get_bag
	?goal <- (objetive get_bag ?name ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Get bag" crlf)
	(assert (plan (name ?name) (number 1)(actions get_bag)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status man went)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_follow_to_taxi
	?goal <- (objetive follow_to_taxi ?name ?place ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Follow to taxi" crlf)
	(assert (plan (name ?name) (number 1)(actions make_task ?name man went)(actions_num_params 2 2)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions follow_to_taxi man ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_followed)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_get_abspos_object
	?goal <- (objetive get_abs_obj ?name ?object ?place ?abspos ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Abspos Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions ask_for ?object ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to ?object)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend ?object)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-pos-object ?place ?abspos)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions make_task ?name ?object finded)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions move manipulator )(duration 6000))) 
	(assert (plan (name ?name) (number 7)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 7))
)

(defrule plan_get_relpos_object
	?goal <- (objetive get_rel_obj ?name object ?place ?relpos ?object ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Relpos Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions ask_for object ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to object)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend object)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-pos-object ?place ?relpos ?object)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions make_task ?name object finded)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions move manipulator )(duration 6000))) 
	(assert (plan (name ?name) (number 7)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 7))
)

(defrule plan_get_oprop_object
	?goal <- (objetive get_oprop_obj ?name ?place ?oprop ?category ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Relpos Object Task" crlf)
	(assert (plan (name ?name) (number 1)(actions ask_for object ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions go_to object)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions attend object)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-pos-object ?place ?oprop ?category)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions make_task ?name object finded)(actions_num_params 6 6)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions move manipulator )(duration 6000))) 
	(assert (plan (name ?name) (number 7)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 7))
)

(defrule plan_find_three_oprop_object
	?goal <- (objetive find_three_oprop_obj ?name ?oprop ?category three ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN find three oprop objects" crlf)
	(assert (plan (name ?name) (number 1)(actions property_object ?oprop ?category three)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status finish_objetive finaly_finded)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_find_three_cat
	?goal <- (objetive find_three_cat ?name ?category ?room three ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Three Category Object Task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?category ?room)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions review_room ?category ?room)(actions_num_params 3 3 3)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_finded)(duration 6000)))
        (assert (finish-planner ?name 3))
)

(defrule plan_find_person_in_door
	?goal <- (objetive find_person_in_door ?name ?person ?place ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-person specific ?person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_went)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_introduce_person
	?goal <- (objetive introduce_person ?name ?p ?person ?php ?place ?step)
	=>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Person Task" crlf)
	(bind ?speech(str-cat "I am sorry, I could not find " ?person))
	(assert (plan (name ?name) (number 1)(actions make_task_neg ?name ?person went)(actions_num_params 2 2)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name ?person went)(actions_num_params 4 4)(duration 6000)))
	;(assert (plan (name ?name) (number 4)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions introduce-person ?p ?person ?php ?place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status finish_objetive finaly_introduced)(duration 6000)))
	(assert (finish-planner ?name 5))
)

(defrule plan_make_question_leave
	?goal <- (objetive make_question ?name ?person leave ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Justina make a question" crlf)
	(bind ?q(str-cat "Hello,_do_you_want_to_leave_the_house"))
	(bind ?confirmation(str-cat "do_you_really_want_to_leave"))
	(bind ?speech(str-cat "I am sorry, I could not find the person"))
	(assert (plan (name ?name) (number 1)(actions make_task_neg ?name ?person went)(actions_num_params 2 2)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name ?person went) (actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions make_question leave ?q ?confirmation)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status finish_objetive finaly_asked)(duration 6000)))
	(assert (finish-planner ?name 5))
)

(defrule plan_guide_to_taxi
	?goal <- (objetive guide_to_taxi ?name ?person ?question ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Justina make a question" crlf)
	(bind ?speech(str-cat "I am sorry, I could not find the person"))
	(assert (plan (name ?name) (number 1)(actions make_task ?name ?person went) (actions_num_params 2 2)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions guide_to_taxi ?person ?question)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_guided)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_clean_up
	?goal <- (objetive clean_up ?name ?room ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Justina clean up" crlf)
	(assert (plan (name ?name) (number 1)(actions clean_up ?room)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status finish_objetive finaly_cleaned)(duration 6000)))
	(assert (finish-planner ?name 2))
)

(defrule plan_take_out_the_garbage
	?goal <- (objetive take_out_garbage ?name ?garbage ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Justina take ou the garbage" crlf)
	(assert (plan (name ?name) (number 1)(actions take_out_garbage ?garbage)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status finish_objetive finaly_taked_out)(duration 6000)))
	(assert (finish-planner ?name 2))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RULES BEFORE SPLIT IN SUBTAREAS


(defrule exe_scheduled-get-object-room 
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (task pgobjroom ?obj ?room ?step)
        =>
        (retract ?f1)
        (assert (objetive gobjroom task_gobjroom ?obj ?room ?step))
)

(defrule exe_scheduled-get-bag
	(state (name ?name)(number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_bag ?step)
	=>
	(retract ?f1)
	(assert (objetive get_bag task_get_bag ?step))
)

(defrule exe_scheduled-follow-to-taxi
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_to_taxi ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_to_taxi task_follow_to_taxi ?place ?step))
)

(defrule exe_scheduled-get-abspos-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_abspos_obj ?object ?place ?abspos ?step)
	=>
	(retract ?f1)
	(assert (objetive get_abs_obj task_get_abs_obj ?object ?place ?abspos ?step))
)

(defrule exe_scheduled-get-relpos-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_relpos_obj object ?place ?relpos ?object ?step)
	=>
	(retract ?f1)
	(assert (objetive get_rel_obj task_get_rel_obj object ?place ?relpos ?object ?step))
)

(defrule exe_scheduled-get-oprop-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_oprop_obj ?place ?oprop ?category ?step)
	=>
	(retract ?f1)
	(assert (objetive get_oprop_obj task_get_oprop_obj ?place ?oprop ?category ?step))
)

(defrule exe_scheduled-find-three-oprop-object 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfind_three_oprop_obj ?oprop ?category three ?step)
	=>
	(retract ?f1)
	(assert (objetive find_three_oprop_obj task_find_three_oprop_obj ?oprop ?category three ?step))
)

(defrule exe_scheduled-find-three-cat 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfind_three_cat ?category ?place three ?step)
	=>
	(retract ?f1)
	(assert (objetive find_three_cat task_find_three_cat ?category ?place three ?step))
)

(defrule exe_scheduled-find-person-in-door 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfind_person_in_door ?person ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive find_person_in_door task_find_person_in_door ?person ?place ?step))
)

(defrule exe_scheduled-introduce-person 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pintroduce_person ?p ?person ?php ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive introduce_person task_introduce_person ?p ?person ?php ?place ?step))
)

(defrule exe_scheduled-make-question  
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pmake_question ?person ?question ?step)
	=>
	(retract ?f1)
	(assert (objetive make_question task_make_question ?person ?question ?step))
)

(defrule exe_scheduled-guide-to-taxi 
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_to_taxi ?person ?question ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_to_taxi task_guide_to_taxi ?person ?question ?step))
)

(defrule exe_scheduled-clean-up
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pclean_up ?room ?step)
	=>
	(retract ?f1)
	(assert (objetive clean_up task_clean_up ?room ?step))
)

(defrule exe_scheduled-take-out-the-garbage
	(state (name ?name) (number ?step) (status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task ptake_out_garbage ?garbage ?step)
	=>
	(retract ?f1)
	(assert (objetive take_out_garbage task_take_out_garbage ?garbage ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
