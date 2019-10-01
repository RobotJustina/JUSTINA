(defrule get_def_obj_location
	?f <- (get_obj_default_loc ?obj 1)
	(item (name ?obj) (zone ?def_loc))
	=>
	(retract ?f)
	(printout t "" ?def_loc "")
)

(defrule set_def_obj_location
	?f <- (set_obj_default_loc ?obj ?zone 1)
	?f1 <- (item (name ?obj))
	=>
	(retract ?f)
	(modify ?f1 (zone ?zone))
)

(defrule no_obj_on_table
	(not(item (type object) (name ?obj) (zone table)))
	=>
	(assert (no obj on_table))
)

(defrule reset_obj_on_table
	?f <- (reset_objs 1)
	(no obj on_table)
	=>
	(retract ?f)
	(assert (start reset_obj))
)

(defrule start_reset_obj
	?f <- (start reset_obj)
	?f1 <- (item (type object)(name ?obj) (zone ?zone&:(neq ?zone table)))
	=>
	(retract ?f)
	(modify ?f1 (zone table))
	(assert (start reset_obj))
)

(defrule finish_restart_obj 
	?f <- (start reset_obj)
	(not (item (name ?obj) (zone no_on_table)))
	=>
	(retract ?f)
)
;;;;
