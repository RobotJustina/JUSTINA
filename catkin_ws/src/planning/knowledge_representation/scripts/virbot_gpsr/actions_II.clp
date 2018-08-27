;********************************************************
;*                                                      *
;*      actions_II.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      23/05/2017                      *
;*                                                      *
;********************************************************


;get_object             params 2        (?object default_location)
;find_object_in_room    parmas 2        (?object ?room)

(defrule task_get_object_def_loc
        ?f <- (task ?plan get_object ?param1 default_location ?step)
        ?f1 <- (item (name ?param1)(type Objects)(zone ?place))

        =>
        (retract ?f)
        (printout t "Get object" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?param1 status grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pgetobj_defloc) (actor robot)(obj robot)(from ?place)(to ?param1)(name-scheduled ?plan)(state-number ?step)))
        ;;;;;;;;;;;
        (modify ?f1 (status nil))       
)

(defrule task_find_object_in_room
        ?f <- (task ?plan find_object_in_room ?object ?room ?step)
        ?f1 <- (item (name ?object)(type Objects))
        (item (name ?room) (type Room))
        =>
        (retract ?f)
        (printout t "Find object in room" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?object status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pfindobj_room) (actor robot)(obj robot)(from ?room)(to ?object)(name-scheduled ?plan)(state-number ?step)))
        ;;;;;;;;;;;
        (modify ?f1 (status nil))
)

(defrule task_find_category_room
        ?f <- (task ?plan find_category_room ?category ?room ?step)
        ?f1 <- (item (name ?category) (type Category))
        (item (name ?room) (type Room))
        =>
        (retract ?f)
        (printout t "Find category in room" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?category status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pfindobj_room) (actor robot)(obj robot)(from ?room)(to ?category)(name-scheduled ?plan)(state-number ?step)))
        ;;;;;;;;;;;
        (modify ?f1 (status nil))
)

(defrule task_how_many_obj
        ?f <- (task ?plan find_how_many_objects ?object ?place ?step)
        ?f1<- (item (name ?object))
        =>
        (retract ?f)
        (printout t "How many objects in some place" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?object status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pmany_obj) (actor robot)(obj robot)(from ?place)(to ?object)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_how_many_cat
        ?f <- (task ?plan find_how_many_category ?category ?place ?step)
        ?f1<- (item (name ?category))
        =>
        (retract ?f)
        (printout t "How many category there are on the place" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?category status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pmany_cat) (actor robot)(obj robot)(from ?place)(to ?category)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_prop_object
        ?f <- (task ?plan find_prop_object ?property ?category ?step)
        ?f1<- (item (name ?property))
        =>
        (retract ?f)
        (printout t "What is the oprop object on the placement" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?property status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pprop_obj) (actor robot)(obj robot)(from ?category)(to ?property)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_gesture_person
        ?f <- (task ?plan find_pgg_person ?gesture ?place ?step)
        ?f1<- (item (name ?gesture))
        =>
        (retract ?f)
        (printout t "Find the gesture person" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?gesture status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pgesture_person) (actor robot)(obj robot)(from ?place)(to ?gesture)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_gender_pose_person
        ?f <- (task ?plan find_gender_pose_person ?posegender ?place ?step)
        ?f1<- (item (name ?posegender))
        =>
        (retract ?f)
        (printout t "Find the gesture person" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?posegender status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pgenderpose_person) (actor robot)(obj robot)(from ?place)(to ?posegender)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
)

(defrule task_gender_pose_crowd
        ?f <- (task ?plan find_gender_pose_crowd ?posegender ?place ?step)
        ?f1<- (item (name ?posegender))
        =>
        (retract ?f)
        (printout t "Find the gesture person" crlf)
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?posegender status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pgenderpose_crowd) (actor robot)(obj robot)(from ?place)(to ?posegender)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
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

(defrule task_ask_for_incomplete
	?f <- (task ?plan ask_info ?nti ?incomplete ?step)
	?f1 <- (item (type question) (name ?nti))
	=>
	(retract ?f)
	(printout t "Task in order to ask for incomplete info")
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?nti status asked)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pask_info) (actor robot)(obj robot)(from ?nti)(to ?incomplete)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
	
)

(defrule task_ask_for_incomplete
	?f <- (task ?plan ask_info ?nti ?incomplete ?param ?step)
	?f1 <- (item (type question) (name ?nti))
	=>
	(retract ?f)
	(printout t "Task in order to ask for incomplete information")
        (assert (state (name ?plan)(number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?nti status asked)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (cd-task (cd pask_info) (actor robot)(obj ?param)(from ?nti)(to ?incomplete)(name-scheduled ?plan)(state-number ?step)))
        (modify ?f1 (status nil))
	
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;  SPLIT IN SUBTAREAS  

(defrule plan_find_object_in_room
        ?goal <- (objetive find_obj_room ?name ?object ?room ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Object Task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?object ?room)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions review_room ?object ?room)(duration 6000)))
        (assert (finish-planner ?name 100))
)

(defrule plan_category_object
        ?goal <- (objetive find_cat_obj ?name ?category ?room ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find Category in Room" crlf)

)

(defrule plan_how_many_obj
        ?goal <- (objetive how_many_obj ?name ?object ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN How many Objects Task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?object ?place)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions go_to ?object)(duration 6000)))
        (assert (plan (name ?name) (number 3)(actions how_many_obj ?object)(duration 6000)))
        (assert (finish-planner ?name 3))
)

(defrule plan_how_many_cat
        ?goal <- (objetive how_many_cat ?name ?category ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN How many CAtegory task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?category ?place)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions go_to ?category)(duration 6000)))
        (assert (plan (name ?name) (number 3)(actions how_many_cat ?category)(duration 6000)))
        (assert (finish-planner ?name 3))
)

(defrule plan_prop_object
        ?goal <- (objetive prop_obj ?name ?property ?category ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan the property of some category" crlf)
        ;(assert (plan (name ?name) (number 1)(actions ask_for ?property ?place)(duration 6000)))
        ;(assert (plan (name ?name) (number 2)(actions go_to ?property)(duration 6000)))
        (assert (plan (name ?name) (number 1)(actions property_object ?property ?category)(duration 6000)))
        (assert (finish-planner ?name 1))
)

(defrule plan_gesture_person
        ?goal <- (objetive gesture_person ?name ?gesture ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan the gesture of some person" crlf)
        (assert (plan (name ?name) (number 1)(actions gesture_person ?gesture ?place)(duration 6000)))
        (assert (finish-planner ?name 1))
)

(defrule plan_gender_pose_person
        ?goal <- (objetive genderpose_person ?name ?posegender ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan the gesture of some person" crlf)
        (assert (plan (name ?name) (number 1)(actions genderpose_person ?posegender ?place)(duration 6000)))
        (assert (finish-planner ?name 1))
)

(defrule plan_gender_pose_crowd
        ?goal <- (objetive genderpose_crowd ?name ?posegender ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan the gesture of people" crlf)
        (assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions genderpose_crowd ?posegender ?place)(duration 6000)))
        (assert (finish-planner ?name 2))
)

(defrule plan_speech_generator
        ?goal <- (objetive speech_generator ?name ?spg ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo Plan speech generator" crlf)
        (assert (plan (name ?name) (number 1)(actions speech_generator ?spg)(duration 6000)))
        (assert (finish-planner ?name 1))
)

(defrule plan_ask_for_incomplete
	?goal <- (objetive ask_for_incomplete ?name ?nti ?incomplete ?plan ?param ?step)
	=>
	(retract ?goal)
	(printout t "Prueba preguntar por informacion incompleta" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for_incomplete ?nti ?incomplete ?plan ?param)(duration 6000)))
        (assert (finish-planner ?name 1))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RULES BEFORE SPLIT IN SUBTAREAS (you can find more rules for split subtareas in the actions.cpl file)

(defrule exe_scheduled-get-object-in-room
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgetobj_defloc) (actor ?robot)(obj ?robot)(from ?place)(to ?param1)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive get_obj task_get ?param1 ?place ?step))
)

(defrule exe_scheduled-find-object-in-room
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfindobj_room) (actor ?robot)(obj ?robot)(from ?room)(to ?object)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_obj_room task_find_object_in_room ?object ?room ?step))
)

(defrule exe_scheduled-how-many-obj
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pmany_obj) (actor ?robot)(obj ?robot)(from ?place)(to ?object)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive how_many_obj task_how_many_obj ?object ?place ?step))
)

(defrule exe_scheduled-how-many-cat
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pmany_cat) (actor ?robot)(obj ?robot)(from ?place)(to ?category)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive how_many_cat task_how_many_cat ?category ?place ?step))
)

(defrule exe_scheduled-prop-object
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pprop_obj) (actor ?robot)(obj ?robot)(from ?category)(to ?property)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive prop_obj task_prop_object ?property ?category ?step))
)

(defrule exe_scheduled-gesture-person
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgesture_person) (actor ?robot)(obj ?robot)(from ?place)(to ?gesture)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive gesture_person task_gesture_person ?gesture ?place ?step))
)

(defrule exe_scheduled-gender-pose-person
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgenderpose_person) (actor ?robot)(obj ?robot)(from ?place)(to ?posegender)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive genderpose_person task_genderpose_person ?posegender ?place ?step))
)

(defrule exe_scheduled-gender-pose-crowd
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pgenderpose_crowd) (actor ?robot)(obj ?robot)(from ?place)(to ?posegender)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive genderpose_crowd task_genderpose_crowd ?posegender ?place ?step))
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

(defrule exe_ask-for-incomplete
        (state (name ?name) (number ?step)(status active)(duration ?time))
        (item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pask_info) (actor ?robot)(obj ?param)(from ?nti)(to ?incomplete)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive ask_for_incomplete task_ask_for_incomplete ?nti ?incomplete ?name ?param ?step))	
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule split_review_room
        ?f  <- (plan (name ?name) (number ?number) (status inactive) (actions review_room ?object ?room))
        ?f1 <- (item (name ?object) (type Objects))
        ?f3 <- (item (name ?place) (status nil)(possession ?room))
        ?f4 <- (num_places ?num)
        =>
        (retract ?f4)
        (assert (visit_place_in_room ?place))
        (assert (num_places (+ ?num 2)))
        (assert (plan (name ?name) (number (+ ?number ?num 1)) (actions go_to_place ?place)) )
        (assert (plan (name ?name) (number (+ ?number ?num 2)) (actions only-find-object ?object ?place)) )
        (modify ?f3 (status prev_review))
)

(defrule split_review_room_cat
        ?f  <- (plan (name ?name) (number ?number) (status inactive) (actions review_room ?category ?room))
        ?f1 <- (item (name ?category) (type Category))
        ?f3 <- (item (name ?place) (status nil) (possession ?room))
        ?f4 <- (num_places ?num)
        =>
        (retract ?f4)
        (assert (visit_place_in_room ?place))
        (assert (num_places (+ ?num 2)))
        (assert (plan (name ?name) (number (+ ?number ?num 1)) (actions go_to_place ?place)) )
        (assert (plan (name ?name) (number (+ ?number ?num 2)) (actions find_cat_obj ?category ?place)) )
        (modify ?f3 (status prev_review))
)

(defrule delate_no_visited_rooms 
        ?f <- (delate_no_visited_rooms ?name )
        ?f1 <- (plan (name ?name) (number ?num) (status inactive) (actions go_to_place ?place))
        ?f2 <- (visit_places ?n1)
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f2)
        (assert (delate_no_visited_rooms ?name))
        (assert (visit_places (+ 1 ?n1)))
)

(defrule delate_no_find_object
        ?f <- (delate_no_visited_rooms ?name)
        ?f1 <- (plan (name ?name) (number ?num) (status inactive) (actions only-find-object ?object ?place))
        ?f2 <- (visit_places ?n1)
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f2)
        (assert (delate_no_visited_rooms ?name))
        (assert (visit_places (+ 1 ?n1)))
)

(defrule delate_no_find_cat
        ?f <- (delate_no_visited_rooms ?name)
        ?f1 <- (plan (name ?name) (number ?num) (status inactive) (actions find_cat_obj ?category ?place))
        ?f2 <- (visit_places ?n1)
        =>
        (retract ?f)
        (retract ?f1)
        (retract ?f2)
        (assert (delate_no_visited_rooms ?name))
        (assert (visit_places (+ 1 ?n1)))
)

(defrule reset_num_visit
        ?f <- (num_places ?n1)
        ?f2 <- (visit_places ?n2&:(eq ?n2 ?n1))
        ?f3 <- (delate_no_visited_rooms ?name)
        =>
        (retract ?f)
        (retract ?f2)
        (retract ?f3)
        (assert (num_places 0))
        (assert (visit_places 0))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; reglas para visitar places pertenecientes a un ROOM


(defrule exe-plan-visit-place
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(actions review_room ?object ?room)(duration ?t))
        =>
        (modify ?f3 (status accomplished))
)


(defrule exe-plan-only-find-object
        (plan (name ?name) (number ?num-pln)(status active)(actions only-find-object ?obj ?place)(duration ?t))
        ?f1 <- (item (name ?obj)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "only_find " ?obj " " ?place ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
        ;(assert (num_places (- ?num 2)))
)

(defrule exe-plan-only-found-object
        ?f <-  (received ?sender command find_object ?object ?x ?y ?z 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions only-find-object ?object ?place))
        ?f3 <- (finish-planner ?name ?n1)
        ?f4 <- (visit_places ?n2)
        =>
        (retract ?f3)
        (retract ?f)
        (retract ?f4)
        (assert (delate_no_visited_rooms ?name))
        (modify ?f2 (status accomplished))
        (modify ?f1 (pose ?x ?y ?z) (status finded))
        (assert (finish-planner ?name ?num-pln))
        (assert (visit_places (+ 2 ?n2)))
)

(defrule exe-plan-no-only-found-object
        ?f <-  (received ?sender command find_object ?object ?x ?y ?z 0)
        ?f1 <- (item (name ?object))
        (num_places ?n1)
        ?f2 <- (plan (name ?name) (number ?num-pln&:(neq ?num-pln (+ 2 ?n1))) (status active)(actions only-find-object ?object ?place))
        ;?f3 <- (finish-planner ?name ?n2)
        ?f3 <- (item (name ?place))
        ?f4 <- (visit_places ?n2)
        =>
        (retract ?f)
        (retract ?f4)
        (modify ?f2 (status accomplished))
        ;(assert (finish-planner ?name ?num-pln))
        (modify ?f3 (status nil))
        (assert (visit_places (+ 2 ?n2)))
)

(defrule exe-plan-no-only-found-object-final
        ?f <-  (received ?sender command find_object ?object ?x ?y ?z 0)
        ?f1 <- (item (name ?object))
        ?f4 <- (num_places ?n1)
        ?f2 <- (plan (name ?name) (number ?num-pln&:(eq ?num-pln (+ 2 ?n1)))(status active)(actions only-find-object ?object ?place))
        ?f3 <- (finish-planner ?name ?n2)
        ?f5 <- (item (name ?place))
        ?f6 <- (visit_places ?n3)
        =>
        (retract ?f)
        (retract ?f3)
        (retract ?f4)
        (retract ?f6)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
        (assert (finish-planner ?name ?num-pln))
        (assert (num_places 0))
        (modify ?f5 (status nil))
        (assert (visit_places 0))
)


(defrule exe-plan-find-cat
        (plan (name ?name) (number ?num-pln)(status active)(actions find_cat_obj ?category ?place)(duration ?t))
        ?f1 <- (item (name ?category)(status ?x&:(neq ?x finded)))
        ?f2 <- (item (name ?place))
        =>
        (bind ?command (str-cat "" ?category " " ?place ""))
        (assert (send-blackboard ACT-PLN find_category ?command ?t 4))
        ;(modify ?f2 (status nil))
)

(defrule exe-plan-found-cat
        ?f <-  (received ?sender command find_category ?category ?place ?cantidad 1)
        ?f1 <- (item (name ?category))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find_cat_obj ?category ?place))
        ?f3 <- (finish-planner ?name ?n1)
        ?f4 <- (visit_places ?n2)
        =>
        (retract ?f3)
        (retract ?f)
        (retract ?f4)
        (assert (delate_no_visited_rooms ?name))
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
        (assert (finish-planner ?name ?num-pln))
        (assert (visit_places (+ 2 ?n2)))
)

(defrule exe-plan-no-found-cat
        ?f <-  (received ?sender command find_category ?category ?place ?cantidad 0)
        ?f1 <- (item (name ?category))
        (num_places ?n1)
        ?f2 <- (plan (name ?name) (number ?num-pln&:(neq ?num-pln (+ 2 ?n1))) (status active)(actions find_cat_obj ?category ?place))
        ?f3 <- (item (name ?place))
        ;?f3 <- (finish-planner ?name ?n2)
        ?f4 <- (visit_places ?n2)
        =>
        (retract ?f)
        (retract ?f4)
        (modify ?f2 (status accomplished))
        (modify ?f3 (status nil))
        (assert (visit_places (+ 2 ?n2)))
        ;(assert (finish-planner ?name ?num-pln))
)

(defrule exe-plan-no-found-cat-final
        ?f <-  (received ?sender command find_category ?category ?place ?cantidad 0)
        ?f1 <- (item (name ?category))
        ?f4 <- (num_places ?n1)
        ?f2 <- (plan (name ?name) (number ?num-pln&:(eq ?num-pln (+ 2 ?n1)))(status active)(actions find_cat_obj ?category ?place))
        ?f3 <- (finish-planner ?name ?n2)
        ?f5 <- (item (name ?place))
        ?f6 <- (visit_places ?n3)
        =>
        (retract ?f)
        (retract ?f3)
        (retract ?f4)
        (retract ?f6)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
        (assert (finish-planner ?name ?num-pln))
        (assert (num_places 0))
        (modify ?f5 (status nil))
        (assert (visit_places 0))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-how-many-obj
        (plan (name ?name) (number ?num-pln)(status active)(actions how_many_obj ?obj)(duration ?t))
        ?f1 <- (item (name ?obj)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-blackboard ACT-PLN many_obj ?command ?t 4))
        ;(assert (num_places (- ?num 2)))
)

(defrule exe-plan-af-many-obj
        ?f <-  (received ?sender command many_obj ?obj ?cantidad 1)
        ?f1 <- (item (name ?obj))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions how_many_obj ?obj))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

(defrule exe-plan-neg-many-obj
        ?f <-  (received ?sender command many_obj ?obj ?cantidad 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions how_many_obj ?obj))
        ?f3 <- (item (name robot))
        =>
        (retract ?f)
        (modify ?f2 (statusTwo active))
        
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-how-many-cat
        (plan (name ?name) (number ?num-pln)(status active)(actions how_many_cat ?category)(duration ?t))
        ?f1 <- (item (name ?category)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?category " find"))
        (assert (send-blackboard ACT-PLN find_category ?command ?t 4))
        ;(assert (num_places (- ?num 2)))
)

(defrule exe-plan-af-many-cat
        ?f <-  (received ?sender command find_category ?category ?param ?cantidad 1)
        ?f1 <- (item (name ?category))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions how_many_cat ?category))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

(defrule exe-plan-neg-many-cat
        ?f <-  (received ?sender command find_category ?category ?param ?cantidad 0)
        ?f1 <- (item (name ?category))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions how_many_cat ?category))
        ?f3 <- (item (name robot))
        =>
        (retract ?f)
        ;(modify ?f2 (statusTwo active))
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-property-object
        (plan (name ?name) (number ?num-pln)(status active)(actions property_object ?property ?category)(duration ?t))
        ?f1 <- (item (name ?property)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?property " " ?category ""))
        (assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-af-property-object
        ?f <-  (received ?sender command prop_obj ?property ?category 1)
        ?f1 <- (item (name ?property))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions property_object ?property ?category))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

(defrule exe-plan-neg-property-object
        ?f <-  (received ?sender command prop_obj ?property ?category 0)
        ?f1 <- (item (name ?property))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions property_object ?property ?category))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-gesture-person
        (plan (name ?name) (number ?num-pln)(status active)(actions gesture_person ?gesture ?place)(duration ?t))
        ?f1 <- (item (name ?gesture)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?gesture " " ?place ""))
        (assert (send-blackboard ACT-PLN gesture_person ?command ?t 4))
)

(defrule exe-plan-af-gesture-person
        ?f <-  (received ?sender command gesture_person ?gesture ?place 1)
        ?f1 <- (item (name ?gesture))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions gesture_person ?gesture ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

(defrule exe-plan-neg-gesture-person
        ?f <-  (received ?sender command gesture_person ?gesture ?place 0)
        ?f1 <- (item (name ?gesture))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions gesture_person ?gesture ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule exe-plan-gender-pose-person
        (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_person ?posegender ?place)(duration ?t))
        ?f1 <- (item (name ?posegender)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?posegender " " ?place ""))
        (assert (send-blackboard ACT-PLN gender_pose_person ?command ?t 4))
)

(defrule exe-plan-af-gender-pose-person
        ?f <-  (received ?sender command gender_pose_person ?posegender ?place 1)
        ?f1 <- (item (name ?posegender))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_person ?posegender ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)


(defrule exe-plan-neg-gender-pose-person
        ?f <-  (received ?sender command gender_pose_person ?posegender ?place 0)
        ?f1 <- (item (name ?posegender))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_person ?posegender ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-gender-pose-crowd
        (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_crowd ?posegender ?place)(duration ?t))
        ?f1 <- (item (name ?posegender)(status ?x&:(neq ?x finded)))
        =>
        (bind ?command (str-cat "" ?posegender " " ?place ""))
        (assert (send-blackboard ACT-PLN gender_pose_crowd ?command ?t 4))
)

(defrule exe-plan-af-gender-pose-crowd
        ?f <-  (received ?sender command gender_pose_crowd ?posegender ?place 1)
        ?f1 <- (item (name ?posegender))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_crowd ?posegender ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)

(defrule exe-plan-neg-gender-pose-crowd
        ?f <-  (received ?sender command gender_pose_crowd ?posegender ?place 0)
        ?f1 <- (item (name ?posegender))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions genderpose_crowd ?posegender ?place))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status finded))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-speech-generator
        (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator ?spg)(duration ?t))
        ?f1 <- (item (name ?name1)(status ?x&:(neq ?x said)) (image ?spg))
        =>
        (bind ?command (str-cat "" ?spg ""))
        (assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-af-speech-generator
        ?f <-  (received ?sender command spg_say ?spg 1)
        ?f1 <- (item (name ?name1) (image ?spg))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator ?spg))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status said))
)

(defrule exe-plan-neg-speech-generator
        ?f <-  (received ?sender command spg_say ?spg 0)
        ?f1 <- (item (name ?name1) (image ?spg))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions speech_generator ?spg))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (status said))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-ask-for-incomplete
	(plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti ?incomplete ?plan ?param)(duration ?t))
	?f1 <- (item (name ?nti) )
	=>
	(bind ?command (str-cat "" ?incomplete " " ?plan " " ?param ""))
	(assert (send-blackboard ACT-PLN ask_incomplete ?command ?t 4))
)

(defrule exe-plan-af-ask-incomplete_follow
	?f <- (received ?sender command ask_incomplete follow_place_origin ?plan ?param ?response 1)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti follow_place_origin ?plan ?param))
	?f3 <- (task ?plan find_person_in_room ?person ?step)
	=>
	(retract ?f)
	(retract ?f3)
	(assert (task ?plan find_person_in_room ?person ?response ?step))
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
)

(defrule exe-plan-af-ask-incomplete_gesture
	?f <- (received ?sender command ask_incomplete gesture_place_origin ?plan ?param ?response 1)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti gesture_place_origin ?plan ?param))
        ?f3 <- (cd-task (cd pgesture_person) (actor robot)(obj robot)(from exitdoor)(to ?gesture)(name-scheduled ?plan)(state-number ?step))
	?f4 <- (task ?plan update_object_location temp place_loc ?step2)
	=>
	(retract ?f)
	(retract ?f4)
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
	(assert (task ?plan update_object_location location ?response ?step2))
)

(defrule exe-plan-af-ask-incomplete_destiny_place
	?f <- (received ?sender command ask_incomplete place_destiny ?plan ?param ?response 1)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti place_destiny ?plan ?param))
	?f3 <- (task ?plan get_object man_guide ?step)
	=>
	(retract ?f)
	(retract ?f3)
	(assert (task ?plan get_object man_guide ?response ?step))
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
)

(defrule exe-plan-af-ask-incomplete_destiny_place_two
	?f <- (received ?sender command ask_incomplete place_destiny ?plan ?param ?response 1)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti place_destiny ?plan ?param))
	?f3 <- (task ?plan get_object man_guide ?place ?step)
	=>
	(retract ?f)
	(retract ?f3)
	(assert (task ?plan get_object man_guide ?response ?step))
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
)

(defrule exe-plan-af-ask-incomplete_object
	?f <- (received ?sender command ask_incomplete object ?plan ?param ?response 1)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti object ?plan ?param))
	?f3 <- (task ?plan get_object ?location ?step)
	?f4 <- (task ?plan handover_object default_location ?step2)
	=>
	(retract ?f)
	(retract ?f3)
	(retract ?f4)
	(assert (task ?plan get_object ?response ?location ?step))
	(assert (task ?plan handover_object ?response ?step2))
	(modify ?f2 (status accomplished))
	(modify ?f1 (status asked))
)

(defrule exe-plan-neg-ask-incomplete
	?f <- (received ?sender command ask_incomplete ?incomplete ?plan ?param ?response 0)
	?f1 <- (item (name ?nti))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions ask_for_incomplete ?nti ?incomplete ?plan ?param))
	=>
	(retract ?f)
	(modify ?f2 (status active))
)


