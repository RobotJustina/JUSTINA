;********************************************************
;*                                                      *
;*      exec_kdb.clp 	                                *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      11/Enero/2019                   *
;*                                                      *
;********************************************************

;;;;;;;;;;;;;;;;;; rules for set the new location of some object
(defrule exe-set-existing-object-room
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_object_location ?obj ?location))
	?f1 <- (item (name ?obj))
	?f2 <- (item (type Room) (name ?location))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "Ok_I_can_find_the_" ?obj "_in_the_" ?location ))
	(modify ?f3 (image ?speech))
	(modify ?f1 (room ?location))
	(modify ?f (status accomplished))
)

(defrule exe-set-existing-object-furniture 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_object_location ?obj ?location))
	?f1 <- (item (name ?obj))
	?f2 <- (item (type Furniture) (name ?location) (room ?room))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "Ok_I_can_find_the_" ?obj "_in_the_" ?location ))
	(modify ?f3 (image ?speech))
	(modify ?f1 (zone ?location)(room ?room))
	(modify ?f (status accomplished))
)

(defrule exe-set-no-existing-object-room
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_object_location ?obj ?location))
	(not (item (name ?obj)))
	?f2 <- (item (type Room) (name ?location))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "Ok_I_can_find_the_" ?obj "_in_the_" ?location ))
	(modify ?f3 (image ?speech))
	(assert (item (type Objects) (name ?obj) (image ?obj) (pose 0.0 0.0 0.0)(room ?location)))
	(modify ?f (status accomplished))
)

(defrule exe-set-no-existing-object-furnitue
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_object_location ?obj ?location))
	(not (item (name ?obj)))
	?f2 <- (item (type Furniture) (name ?location)(room ?room))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "Ok_I_can_find_the_" ?obj "_in_the_" ?location ))
	(modify ?f3 (image ?speech))
	(assert (item (type Objects) (name ?obj) (image ?obj) (pose 0.0 0.0 0.0)(zone ?location)(room ?room)))
	(modify ?f (status accomplished))
)

(defrule exe-not-set-object-location
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_object_location ?obj ?location))
	(not (item (type Room) (name ?location)))
	(not (item (type Furniture) (name ?location)))
	?f1 <- (item (name speech))
	=>
	(bind ?speech(str-cat "I_am_sorry,_I_dont_know_where_the_" ?location "_is" ))
	(modify ?f1 (image ?speech))
	(modify ?f (status accomplished))
)

;;;;;;;;;;;;;;;;;rules for get the registred location of some object

(defrule exe-get-existing-object-room
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_object_location ?obj))
	?f1 <- (item (name ?obj)(zone ?zone&:(neq ?zone nil)) (room ?room&:(neq ?room nil)))
	?f2 <- (item (name ?zone) (room ?room))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "The_" ?obj "_is_on_the_" ?zone "_in_the_" ?room ))
	(modify ?f3 (image ?speech))
	(modify ?f (status accomplished))
)

(defrule exe-get-existing-object-room-not-furniture 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_object_location ?obj))
	?f1 <- (item (name ?obj)(zone ?zone) (room ?room&:(neq ?room nil)))
	(not (item (name ?zone) (room ?room)))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "The_" ?obj "_is_in_the_" ?room ))
	(modify ?f3 (image ?speech))
	(modify ?f (status accomplished))
)

(defrule exe-get-existing-object-room-nil-furniture 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_object_location ?obj))
	?f1 <- (item (name ?obj)(zone nil) (room ?room&:(neq ?room nil)))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "The_" ?obj "_is_in_the_" ?room ))
	(modify ?f3 (image ?speech))
	(modify ?f (status accomplished))
)

(defrule exe-get-not-existing-object
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_object_location ?obj))
	(not (item (name ?obj)))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "I_dont_know_where_the_" ?obj "_is" ))
	(modify ?f3 (image ?speech))
	(modify ?f (status accomplished))
)

(defrule exe-get-not-existing-object-location
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_object_location ?obj))
	(item (name ?obj)(zone nil)(room nil))
	?f3 <- (item (name speech))
	=>
	(bind ?speech(str-cat "I_dont_know_where_the_" ?obj "_is" ))
	(modify ?f3 (image ?speech))
	(modify ?f (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
