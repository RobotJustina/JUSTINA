;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;		19/03/2018		;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule get_category
	?f <- (cmd_get_category ?obj ?ind ?conf 1)
	?f1 <- (item (name ?obj) (category ?cat) (grasp ?priority))
	=>
	(retract ?f)
	(printout t ?cat)
	(assert (set_obj_to_grasp ?obj (* ?conf ?priority) ?priority ?ind))
)

(defrule set_obj_to_grasp
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	(not (obj_to_grasp ?name ?r2 ?p2 ?ind2 ?num))
	=>
	(retract ?f)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 1))
)

(defrule set_obj_to_grasp_lower
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(< ?result ?r1) ?p1 ?ind1 1)
	(not (obj_to_grasp ?name ?r2 ?p2 ?ind2 2))
	=>
	(retract ?f)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 2))
)

(defrule set_obj_to_grasp_upper
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(> ?result ?r1) ?p1 ?ind1 1)
	(not (obj_to_grasp ?name ?r2 ?p2 ?ind2 2))
	=>
	(retract ?f ?f1)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 1))
	(assert (obj_to_grasp ?obj1 ?r1 ?p1 ?ind1 2))
	
)

(defrule set_obj_to_grasp_equal_lower
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(eq ?r1 ?result) ?p1&:(> ?p1 ?priority) ?ind1 1)
	(not (obj_to_grasp ?name ?r2 ?p2 ?ind2 2))
	=>
	(retract ?f)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 2)) 
)

(defrule set_obj_to_grasp_equal_upper
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(eq ?r1 ?result) ?p1&:(< ?p1 ?priority) ?ind1 1)
	(not (obj_to_grasp ?name ?r2 ?p2 ?ind2 2))
	=>
	(retract ?f ?f1)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 1))
	(assert (obj_to_grasp ?obj1 ?r1 ?p1 ?ind1 2))
	
)

;;;;;

(defrule set_obj_to_grasp_thrird
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(< ?result ?r1) ?p1 ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2&:(< ?result ?r2) ?p2 ?ind2 2)
	=>
	(retract ?f)
)

(defrule set_obj_to_grasp_upper_second
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(< ?result ?r1) ?p1 ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2&:(> ?result ?r2) ?p2 ?ind2 2)
	=>
	(retract ?f ?f2)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 2))
)

(defrule set_obj_to_grasp_upper_first
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(> ?result ?r1) ?p1 ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2&:(> ?result ?r2) ?p2 ?ind2 2)
	=>
	(retract ?f ?f1 ?f2)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 1))
	(assert (obj_to_grasp ?obj1 ?r1 ?p1 ?ind1 2))
)

(defrule set_obj_to_grasp_equal_both_second
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(eq ?result ?r1) ?p1&:(< ?priority ?p1) ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2&:(eq ?result ?r2) ?p2&:(> ?priority ?p2) ?ind2 2)
	=>
	(retract ?f ?f2)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 2))
)

(defrule set_obj_to_grasp_equal_both_first 
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(eq ?result ?r1) ?p1&:(> ?priority ?p1) ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2 ?p2 ?ind2 2)
	=>
	(retract ?f ?f1 ?f2)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 1))
	(assert (obj_to_grasp ?obj1 ?r1 ?p1 ?ind1 2))
)

(defrule set_obj_to_grasp_equal_second
	?f <- (set_obj_to_grasp ?obj ?result ?priority ?ind)
	?f1 <- (obj_to_grasp ?obj1 ?r1&:(< ?result ?r1) ?p1 ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2&:(eq ?result ?r2) ?p2&:(> ?priority ?p2) ?ind2 2)
	=>
	(retract ?f ?f2)
	(assert (obj_to_grasp ?obj ?result ?priority ?ind 2)) 
)

;;;;;;;;;;;;;;;;;

(defrule get_two_objects_to_grasp
	?f <- (cmd_get_objects_to_grasp 1)
	?f1 <- (obj_to_grasp ?obj1 ?r1 ?p1 ?ind1 1)
	?f2 <- (obj_to_grasp ?obj2 ?r2 ?p2 ?ind2 2)
	=>
	(retract ?f ?f1 ?f2)
	(bind ?command(str-cat "" ?ind1 " " ?ind2 ""))
	(printout t ?command)
)

;;;;
