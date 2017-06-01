;********************************************************
;*                                                      *
;*      schedule_cubes.clp                              *
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
        (bind ?command (str-cat "only_find " ?obj ""))
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
        (bind ?command (str-cat "" ?category " find"))
        (assert (send-blackboard ACT-PLN find_category ?command ?t 4))
        ;(modify ?f2 (status nil))
)

(defrule exe-plan-found-cat
        ?f <-  (received ?sender command find_category ?category find ?cantidad 1)
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
        ?f <-  (received ?sender command find_category ?category ?param1 ?cantidad 0)
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
        ?f <-  (received ?sender command find_category ?category ?param1 ?cantidad 0)
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

