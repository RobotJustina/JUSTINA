;********************************************************
;*							*
;*							*
;*							*
;*			University of Mexico
;*			Julio Cesar Cruz Estrda		*
;*							*
;*			28/08/2018			*
;*							*
;********************************************************

;;;;;;;;;;;; update the changes in Getting to know my home Task
(defrule door_status
	?f <- (cmd_iros_update door ?name ?room1 ?room2 ?state)
	;?f1 <- (Door (name ?name)(room_first ?room1)(room_second ?room2))
	?f2 <- (item (name semantic_map) (status $?text))
	=>
	(retract ?f)
	(bind ?semanticMap(str-cat " type(" ?name ", door). connects(" ?name ", " ?room1 ", " ?room2 "). isOpen(" ?name ", false)."))
	;(modify ?f1 (status ?state)(num 0))
	(modify ?f2 (status $?text ?semanticMap))
	(printout t "-")
)

(defrule object_status
	?f <- (cmd_iros_update object ?type ?name ?forniture ?img_name)
	?f1 <- (item (type ?type) (zone ?zone))
	?f2 <- (item (name ?forniture)(room ?room))
	?f3 <- (item (name semantic_map) (status $?text))
	=>
	(retract ?f)
	(bind ?semanticMap(str-cat " type(" ?name ", " ?type "). in(" ?name ", " ?room "). on(" ?name ", " ?forniture "). picture(" ?name ", " ?img_name ")."))
	;(modify ?f1 (zone ?forniture) (room ?room) (image ?img_name)(num 0))
	(assert (move_obj ?type ?name from ?forniture to ?zone))
	(modify ?f3 (status $?text ?semanticMap))
	(printout t "-")
)

(defrule forniture_status
	?f <- (cmd_iros_update forniture ?type ?name ?room)
	?f1 <- (item (type ?type))
	?f2 <- (item (name semantic_map) (status $?text))
	=>
	(retract ?f)
	(bind ?semanticMap(str-cat " type(" ?name ", " ?type "). in(" ?name ", " ?room ")."))
	;(modify ?f1 (room ?room)(num 0))
	(modify ?f2 (status $?text ?semanticMap))
	(printout t "-")
)

;;;;;;;;;;;; rules for send the semantic map
(defrule send_semantic_map
	?f <- (cmd_iros_semantic_map 1)
	?f1 <- (item (name semantic_map) (status $?text))
	=> 
	(retract ?f)
	(printout t $?text)
	(modify ?f1 (status _))
)

;;;;;;;;;;;;;;;;;;;;;;;;;; rules for doors conections and doors locations for navigation

(defrule doors_in_a_path
	?f <- (cmd_iros_make_doors_path ?room1 ?room2 $?rest 1)
	;?f1 <- (item (name door_path) (status ))
	=>
	(retract ?f)
	(assert (doors_path ?room1 ?room2 $?rest))
	(printout t "-")
)

(defrule get_door_loc_only_two_rooms
	?f <- (doors_path ?room1 ?room2)
	?f1 <- (Door (name ?name) (room_first ?r&:(or (eq ?r ?room1) (eq ?r ?room2))) (room_second ?r2&:(or (eq ?r2 ?room1) (eq ?r2 ?room2))))
	;?f2 <- (item (name ?room1) (shared ?door_location))
	?f3 <- (item (name door_path) (status $?text))
	=>
	(retract ?f)
	(bind ?door_location(str-cat "door-" ?room1 "-" ?room2))
	(modify ?f3 (status $?text ?door_location))
)

(defrule not_get_door_loc_only_two_rooms
	?f <- (doors_path ?room1 ?room2)
	(not (Door (name ?name) (room_first ?r&:(or (eq ?r ?room1) (eq ?r ?room2))) (room_second ?r2&:(or (eq ?r2 ?room1) (eq ?r2 ?room2)))))
	=>
	(retract ?f)
)

(defrule get_door_loc
	?f <- (doors_path ?room1 ?room2 ?room3 $?rest)
	?f1 <- (Door (name ?name) (room_first ?r&:(or (eq ?r ?room1) (eq ?r ?room2))) (room_second ?r2&:(or (eq ?r2 ?room1) (eq ?r2 ?room2))))
	;?f2 <- (item (name ?room1) (shared ?door_location))
	?f3 <- (item (name door_path)(status $?text))
	=>
	(retract ?f)
	(bind ?door_location(str-cat "door-" ?room1 "-" ?room2))
	(assert (doors_path ?room2 ?room3 $?rest))
	(modify ?f3 (status $?text ?door_location))
)

(defrule not_get_door_loc
	?f <- (doors_path ?room1 ?room2 ?room3 $?rest)
	(not (Door (name ?name) (room_first ?r&:(or (eq ?r ?room1) (eq ?r ?room2))) (room_second ?r2&:(or (eq ?r2 ?room1) (eq ?r2 ?room2)))))
	=>
	(retract ?f)
	(assert (doors_path ?room2 ?room3 $?rest))
)

(defrule send_doors_path
	?f <- (cmd_iros_get_doors_path 1)
	?f1 <- (item (name door_path) (status $?text))
	=>
	(retract ?f)
	(printout t $?text)
	(modify ?f1 (status _))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; rules to say if an object is in its default position or not

(defrule default_obj_pos
	?f <- (cmd_iros_default_pos ?type ?name ?pos 1)
	?f1 <- (item (type ?type) (zone ?pos))
	=>
	(retract ?f)
	(bind ?command(str-cat "true"))
	(printout t ?command)
)

(defrule no_default_obj_pos
	?f <- (cmd_iros_default_pos ?type ?name ?pos 1)
	?f1 <- (item (type ?type) (zone ?p&: (neq ?p ?pos)))
	=>
	(retract ?f)
	(bind ?command(str-cat "false"))
	(printout t ?command)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;; rules for get object's origin and destiny locations

(defrule get_obj_origin_destiny
	?f <- (cmd_iros_obj_ori_dest ?type ?name 1)
	?f1 <- (move_obj ?type ?name from ?origin to ?destiny)
	=>
	(retract ?f ?f1)
	(bind ?command(str-cat ?origin"-"?destiny))
	(printout t ?command)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
