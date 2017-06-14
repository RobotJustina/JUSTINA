;********************************************************
;*							*
;*							*
;*							*
;*			University of Mexico
;*                      Hugo Leon       		*
;*			Julio Cesar Cruz Estrda		*
;*							*
;*			03/02/2017			*
;*							*
;********************************************************
;
; Scene:
;	1) The robot waits for the instruction
;	


;(deffacts rules_spr

;	(ready spr_kr)

;)

;(defrule exe_cmdSpeech
	
;	?f1 <- (ready spr_kr)
;	 =>
;	(retract ?f1)
;        (printout t "SPR KNOWLEDGE READY" crlf)
;        (assert (cmd_bigger pringles senbei 1))
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;; biiger between two objects

(defrule bigger_two_objects
	?f <- (cmd_compare bigger ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj3) (size ?tam1&:(> ?tam1 ?tam2)))
	=> 
	(retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is bigger"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
	(printout t "El objeto mayor es " ?obj3 " " ?tam1 crlf)
	
)

(defrule bigger_two_objects_2
        ?f <- (cmd_compare bigger ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj3) (size ?tam2&:(> ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is bigger"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto mayor es " ?obj3 " " ?tam2 crlf)
)

;;;;;;;;;;;;;;;;;
;;;;;;;; smaller between two objects

(defrule smaller_two_objects
        ?f <- (cmd_compare smaller ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj3) (size ?tam1&:(< ?tam1 ?tam2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is smaller"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de menor tamanio es " ?obj3 " " ?tam1 crlf)
)

(defrule smaller_two_objects_2
        ?f <- (cmd_compare smaller ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj3) (size ?tam2&:(< ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is smaller"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de menor tamanio es " ?obj3 " " ?tam2 crlf)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;  category of one object
;;;;;;  $objq = To which category belong the {object}?
(defrule category_one_objects
        ?f <- (cmd_category ?obj1 1)
        (item (name ?obj1) (category ?cat))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is a " ?cat))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "La categoria del " ?obj1 " es " ?cat crlf)
)

(defrule simple_category_one_object
	?f <- (cmd_simple_category ?obj1 1)
	(item (name ?obj1) (category ?cat))
	=>
	(retract ?f)
        (bind ?command ?cat)
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "La categoria del " ?obj1 " es " ?cat crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;; same category of two objects
;;;   $objq = Do the {object 1} and {object 2} belong to the same category?

(defrule same_category_two_objects
        ?f <- (cmd_same_category ?obj1 ?obj2 1)
        (item (name ?obj1) (category ?cat1))
        (item (name ?obj2) (category ?cat1))
        => 
        (retract ?f)
        (bind ?command (str-cat  "Yes, the " ?obj1 " and the " ?obj2 " belong to the same category"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?obj1 " tiene la misma cat que el " ?obj2 crlf)
)

(defrule different_category_two_objects
        ?f <- (cmd_same_category ?obj1 ?obj2 1)
        (item (name ?obj1) (category ?cat1))
        (item (name ?obj2) (category ?cat2&:(neq ?cat1 ?cat2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "Not, the " ?obj1 " and the " ?obj2 " do not belong to the same category"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?obj1 " tiene diferente cat que el " ?obj2 crlf)
        
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;  heaviest between two objects

(defrule heavier_two_objects
        ?f <- (cmd_compare heavier ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj3) (weight ?w1&:(> ?w1 ?w2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is heavier"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de mayor peso es " ?obj3 " " ?w1 crlf) 
)

(defrule heavier_two_objects_2
        ?f <- (cmd_compare heavier ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj3) (weight ?w2&:(> ?w2 ?w1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is heavier"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de mayor peso es " ?obj3 " " ?w2 crlf)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;  lightest between two objects

(defrule lightest_two_objects
        ?f <- (cmd_compare lighter ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj3) (weight ?w1&:(< ?w1 ?w2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is lighter"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de menor peso es " ?obj3 " " ?w1 crlf)
        
)

(defrule lightest_two_objects_2
        ?f <- (cmd_compare lighter ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj3) (weight ?w2&:(< ?w2 ?w1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj3 " is lighter"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto de menor peso es " ?obj3 " " ?w2 crlf)
)

;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;   how many objects of some category
;$objq = How many {category} there are?

(defrule haw_many_objects_category
        ?f <- (cmd_many_cat ?cat 1)
        (item (name ?cat) (quantity ?q1))
        => 
        (retract ?f)
        (bind ?command (str-cat  "There are " ?q1 " " ?cat))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "Hay " ?q1 " de la categoria " ?cat crlf)
)

;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;   how many objects

(defrule haw_many_objects
        ?f <- (cmd_many_obj ?obj1 1)
        (item (name ?obj1) (quantity ?q1))
        => 
        (retract ?f)
        (printout t "Hay " ?q1 " " ?obj1 crlf)
)


;;;;;;;;;;;;;;;;;;;
;;;;;;;;   biggest
(defrule biggest
        ?f <- (cmd_absolute_compare biggest 1)
        (item (name ?obj1) (biggest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The biggest object is the " ?obj1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto mas grande es " ?obj1 crlf)
)

(defrule biggest_cat
        ?f <- (cmd_absolute_compare ?cat biggest 1)
        (item (name ?cat) (biggest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The biggest " ?cat " is the " ?obj))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;$objq = Which is the $adja object
;;;;;;;    smallest

(defrule smallest
        ?f <- (cmd_absolute_compare smallest 1)
        (item (name ?obj1) (smallest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The smallest object is the " ?obj1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El objeto mas pequenio es " ?obj1 crlf)
)

(defrule smallest_cat
        ?f <- (cmd_absolute_compare ?cat smallest 1)
        (item (name ?cat) (smallest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The smallest " ?cat " is the " ?obj))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;;;;;;;;;;;;;;;
;;;;;;;;    heaviest

(defrule heaviest
        ?f <- (cmd_absolute_compare heaviest 1)
        (item (name ?obj1) (heaviest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The heaviest object is the " ?obj1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "el objeto mas pesado es " ?obj1 crlf)
)

(defrule heaviest_cat
        ?f <- (cmd_absolute_compare ?cat heaviest 1)
        (item (name ?cat) (heaviest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The heaviest " ?cat " is the " ?obj))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;;;;;;;;;;;;
;;;;;;   lightest

(defrule lightest
        ?f <- (cmd_absolute_compare lightest 1)
        (item (name ?obj1) (lightest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The lightest object is the " ?obj1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
)

(defrule lightest_cat
        ?f <- (cmd_absolute_compare ?cat lightest 1)
        (item (name ?cat) (lightest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The lightest " ?cat " is the " ?obj))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;;;;;;;;;;;;;;
;;;;;;;;     color of some object

(defrule color
        ?f <- (cmd_color ?obj1 1)
        (item (name ?obj1) (color ?c1))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The color of the " ?obj1 " is " ?c1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?obj1 " es de color " ?c1 crlf)
)


;;;;;;;;;;
;;;;;;     where is one object
;$objq = Where can I find a {object}?

(defrule where_is_object
        ?f <- (cmd_where ?obj1 1)
        (item (type Objects)(name ?obj1) (zone ?l1))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is in the " ?l1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?obj1 " se encuentra en " ?l1 crlf)
)

;$objq = Where can I find a {category}?
(defrule where_is_category
        ?f <- (cmd_where ?cat1 1)
        (item (type Category)(name ?cat1) (zone ?l1))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?cat1 " are in the " ?l1))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "The " ?cat1 " se encuentra en " ?l1 crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;; what objects there are in some forniture
;;;$objq = What objects are stored in the {placement}?


(defrule what_category
        ?f <- (cmd_what_obj ?location 1)
        (item (type Category)(name ?cat1) (zone ?location))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?cat1 " are in the " ?location))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "Los " ?cat1 " se encuentran en " ?location crlf)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;  how many ?obj in some ?place
;$objq = How many ({category} | objects) are in the {placement}?

(defrule how_many_cat
        ?f <- (cmd_many_cat_place ?category ?place 1)
        (item (type Category)(name ?category)(zone ?place)(quantity ?q))
        =>
        (retract ?f)
        (bind ?command (str-cat  "There are " ?q " " ?category " in the " ?place))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "Hay " ?q " en el " ?place crlf)
)

(defrule how_many_cat_negative 
        ?f <- (cmd_many_cat_place ?category ?place 1)
        (item (type Category) (name ?c&:(neq ?c ?category)) (zone ?place) (quantity ?q))
        =>
        (retract ?f)
        (bind ?command (str-cat  "There are zero " ?category " in the " ?place))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "Hay 0 " ?category " in the " ?place crlf)
)


(defrule how_many_obj 
        ?f <- (cmd_many_obj_place ?place 1)
        (item (type Category) (zone ?place) (quantity ?q))
        =>
        (retract ?f)
        (bind ?command (str-cat  "There are " ?q " objects in the " ?place))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "Hay " ?q " objects in the " ?place crlf)
        )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;$objq = Which is the $adja {category}?
;;;;;;;    


;$arenaq = Where is the {placement}?
;$arenaq = Where is the {beacon}?
;$arenaq = In which room is the {placement}?
;$arenaq = In which room is the {beacon}?

(defrule where_placement
        ?f <- (cmd_what_place ?place 1)
        (item (name ?place) (room ?location))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?place " is in the " ?location))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?place " se encuentra en " ?location crlf)
)

;$arenaq = How many doors has the {room}?
(defrule how_many_doors
        ?f <- (cmd_many_doors ?room 1)
        (item (type Door)(room ?room)(quantity ?quantity))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " has " ?quantity " doors"))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?room " tiene " ?quantity " doors" crlf)
)

;$arenaq = How many ({placement} | {beacon}) are in the {room}?

(defrule how_many_forniture_afirmative
        ?f <- (cmd_many_for ?forniture ?room 1)
        (item (name ?forniture) (room ?room))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " have one " ?forniture))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?room " tiene 1 " ?forniture crlf)
)

(defrule how_many_forniture_negative
        ?f <- (cmd_many_for ?forniture ?room 1)
        (item (name ?forniture) (room ?r&:(neq ?r ?room)))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " have zero " ?forniture))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t "El " ?room " tiene 0 " ?forniture crlf)
)

;;; ----- Question of Crowd questions -----
;$crowdq = How many $people are in the crowd?
(defrule many_people
	?f <- (cmd_many_people ?people 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " " ?people))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;$crowdq = How many people in the crowd are ($posppl | {gesture})?
(defrule many_people_posprs
	?f <- (cmd_many_people_posprs ?posprs 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " people " ?posprs))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

(defrule many_people_two_posprs
	?f <- (cmd_many_people_posprs ?posprs1 ?posprs2 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " people " ?posprs1 " or " ?posprs2))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

(defrule many_people_gesture
	?f <- (cmd_many_people_gesture ?gesture 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " people " ?gesture))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;$crowdq = Was the $posprs person $gprsng?
(defrule many_people_was_two_gprsn
	?f <- (cmd_many_people_was ?posprs ?gprsn1 ?gprsn2 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 1 2))
	(if (= ?manyPeople 1)
	    	then (bind ?wasPeople ?gprsn1)
	    	else (bind ?wasPeople ?gprsn2)
	)
        (bind ?command (str-cat  "The " ?posprs " person was a " ?wasPeople))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;wdq = Tell me if the ($posprs | {gesture}) person was a $gprsn?
(defrule many_people_was_gprsn
	?f <- (cmd_many_people_was ?posprs ?gprsn 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 1 2))
	(if (= ?manyPeople 1)
	    	then (bind ?wasPeople "Yes" )
	    	else (bind ?wasPeople "Not")
	)
        (bind ?command (str-cat ?wasPeople " , the person " ?posprs " was a " ?gprsn))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

(defrule many_people_was_gesture
	?f <- (cmd_many_people_was_gesture ?gesture ?gprsn 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 1 2))
	(if (= ?manyPeople 1)
	    	then (bind ?wasPeople "Yes" )
	    	else (bind ?wasPeople "Not")
	)
        (bind ?command (str-cat ?wasPeople " , the person " ?gesture " was a " ?gprsn))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)

;$crowdq = Tell me how many people were wearing $color
(defrule many_people_color
	?f <- (cmd_many_people_color ?color 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "There were " ?manyPeople " people wearing " ?color))
        (assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command crlf)
)
