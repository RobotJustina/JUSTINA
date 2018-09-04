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
        ;(item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj1) (size ?tam1&:(>= ?tam1 ?tam2)))
	=> 
	(retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is bigger"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
	(printout t ?command)
	
)

(defrule bigger_two_objects_2
        ?f <- (cmd_compare bigger ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        ;(item (name ?obj2) (size ?tam2))
        (item (name ?obj2) (size ?tam2&:(> ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is bigger"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;;;;;;
;;;;;;;; smaller between two objects

(defrule smaller_two_objects
        ?f <- (cmd_compare smaller ?obj1 ?obj2 1)
        ;(item (name ?obj1) (size ?tam1))
        (item (name ?obj2) (size ?tam2))
        (item (name ?obj1) (size ?tam1&:(<= ?tam1 ?tam2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is smaller"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule smaller_two_objects_2
        ?f <- (cmd_compare smaller ?obj1 ?obj2 1)
        (item (name ?obj1) (size ?tam1))
        ;(item (name ?obj2) (size ?tam2))
        (item (name ?obj2) (size ?tam2&:(< ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is smaller"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; larger between two objects

(defrule larger_two_objects
        ?f <- (cmd_compare larger ?obj1 ?obj2 1)
        ;(item (name ?obj1) (height ?tam1))
        (item (name ?obj2) (height ?tam2))
        (item (name ?obj1) (height ?tam1&:(>= ?tam1 ?tam2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is larger"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule larger_two_objects_2
        ?f <- (cmd_compare larger ?obj1 ?obj2 1)
        (item (name ?obj1) (height ?tam1))
        ;(item (name ?obj2) (height ?tam2))
        (item (name ?obj2) (height ?tam2&:(> ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is larger"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;  thinner between two objects

(defrule thinner_two_objects
        ?f <- (cmd_compare thinner ?obj1 ?obj2 1)
        ;(item (name ?obj1) (wide ?tam1))
        (item (name ?obj2) (wide ?tam2))
        (item (name ?obj1) (wide ?tam1&:(<= ?tam1 ?tam2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is thinner"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule thinner_two_objects_2
        ?f <- (cmd_compare thinner ?obj1 ?obj2 1)
        (item (name ?obj1) (wide ?tam1))
        ;(item (name ?obj2) (wide ?tam2))
        (item (name ?obj2) (wide ?tam2&:(< ?tam2 ?tam1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is thinner"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule simple_category_one_object
	?f <- (cmd_simple_category ?obj1 1)
	(item (name ?obj1) (category ?cat))
	=>
	(retract ?f)
        ;(bind ?command ?cat)
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?cat)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule different_category_two_objects
        ?f <- (cmd_same_category ?obj1 ?obj2 1)
        (item (name ?obj1) (category ?cat1))
        (item (name ?obj2) (category ?cat2&:(neq ?cat1 ?cat2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "Not, the " ?obj1 " and the " ?obj2 " do not belong to the same category"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
        
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;  heaviest between two objects

(defrule heavier_two_objects
        ?f <- (cmd_compare heavier ?obj1 ?obj2 1)
        ;(item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj1) (weight ?w1&:(>= ?w1 ?w2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is heavier"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command) 
)

(defrule heavier_two_objects_2
        ?f <- (cmd_compare heavier ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        ;(item (name ?obj2) (weight ?w2))
        (item (name ?obj2) (weight ?w2&:(> ?w2 ?w1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is heavier"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;  lightest between two objects

(defrule lightest_two_objects
        ?f <- (cmd_compare lighter ?obj1 ?obj2 1)
        ;(item (name ?obj1) (weight ?w1))
        (item (name ?obj2) (weight ?w2))
        (item (name ?obj1) (weight ?w1&:(<= ?w1 ?w2)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj1 " is lighter"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
        
)

(defrule lightest_two_objects_2
        ?f <- (cmd_compare lighter ?obj1 ?obj2 1)
        (item (name ?obj1) (weight ?w1))
        ;(item (name ?obj2) (weight ?w2))
        (item (name ?obj2) (weight ?w2&:(< ?w2 ?w1)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?obj2 " is lighter"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule biggest_cat
        ?f <- (cmd_absolute_compare ?cat biggest 1)
        (item (name ?cat) (biggest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The biggest " ?cat " is the " ?obj))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule smallest_cat
        ?f <- (cmd_absolute_compare ?cat smallest 1)
        (item (name ?cat) (smallest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The smallest " ?cat " is the " ?obj))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;;;;
;;;;;;;;    heaviest

(defrule heaviest
        ?f <- (cmd_absolute_compare heaviest 1)
        (item (name ?obj1) (heaviest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The heaviest object is the " ?obj1))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule heaviest_cat
        ?f <- (cmd_absolute_compare ?cat heaviest 1)
        (item (name ?cat) (heaviest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The heaviest " ?cat " is the " ?obj))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;
;;;;;;   lightest

(defrule lightest
        ?f <- (cmd_absolute_compare lightest 1)
        (item (name ?obj1) (lightest ?b1&:(eq ?b1 yes)))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The lightest object is the " ?obj1))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule lightest_cat
        ?f <- (cmd_absolute_compare ?cat lightest 1)
        (item (name ?cat) (lightest ?obj))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The lightest " ?cat " is the " ?obj))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;;;
;;;;;;;;     color of some object

(defrule color
        ?f <- (cmd_color ?obj1 1)
        (item (name ?obj1) (color ?c1))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The color of the " ?obj1 " is " ?c1))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;$objq = Where can I find a {category}?
(defrule where_is_category
        ?f <- (cmd_where ?cat1 1)
        (item (type Category)(name ?cat1) (zone ?l1))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?cat1 " are in the " ?l1))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;; what objects there are in some forniture
;;;$objq = What objects are stored in the {placement}?


(defrule what_category
        ?f <- (cmd_what_obj ?location 1)
        (item (type Category)(name ?cat1&:(neq ?cat1 nil)) (zone ?location))
        => 
        (retract ?f)
        (bind ?command (str-cat  "The " ?cat1 " are in the " ?location))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule what_category_nil
	?f <- (cmd_what_obj ?location 1)
	(item (type Category)(name nil) (zone ?location))
	=>
	(retract ?f)
	(bind ?command (str-cat "There are not objects stored in the " ?location))
	(printout t ?command)
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
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule how_many_cat_negative 
        ?f <- (cmd_many_cat_place ?category ?place 1)
        (item (type Category) (name ?c&:(neq ?c ?category)) (zone ?place) (quantity ?q))
        =>
        (retract ?f)
        (bind ?command (str-cat  "There are zero " ?category " in the " ?place))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)


(defrule how_many_obj 
        ?f <- (cmd_many_obj_place ?place 1)
        (item (type Category) (zone ?place) (quantity ?q))
        =>
        (retract ?f)
        (bind ?command (str-cat  "There are " ?q " objects in the " ?place))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
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
        (item (type ?fd&:(or (eq ?fd Furniture) (eq ?fd Door))) (name ?place) (room ?location))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?place " is in the " ?location))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule where_placement_room
	?f <- (cmd_what_place ?room 1)
	(item (type Room)(name ?room) (room ?location))
	=>
	(retract ?f)
	(bind ?command (str-cat "Actually the " ?room " is a room of the arena"))
	(printout t ?command)
)

;$arenaq = How many doors has the {room}?
(defrule how_many_doors
        ?f <- (cmd_many_doors ?room 1)
        (item (type Door)(room ?room)(quantity ?quantity))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " has " ?quantity " doors"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;$arenaq = How many ({placement} | {beacon}) are in the {room}?

(defrule how_many_forniture_afirmative
        ?f <- (cmd_many_for ?forniture ?room 1)
        (item (name ?forniture) (room ?room))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " have one " ?forniture))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule how_many_forniture_negative
        ?f <- (cmd_many_for ?forniture ?room 1)
        (item (name ?forniture) (room ?r&:(neq ?r ?room)))
        =>
        (retract ?f)
        (bind ?command (str-cat  "The " ?room " have zero " ?forniture))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;; ----- Question of Crowd questions -----
;$crowdq = How many $people are in the crowd?
(defrule many_people
	?f <- (cmd_many_people ?gender 1)
	(item (name ?gender) (quantity ?manyPeople))
	=>
	(retract ?f)
	;(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " " ?gender))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;$crowdq = How many people in the crowd are ($posppl | {gesture})?
(defrule many_people_posprs
	?f <- (cmd_many_people_posprs ?posprs 1)
	(item (name ?posprs) (quantity ?manyPeople))
	=>
	(retract ?f)
	;(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " people " ?posprs))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule many_people_two_posprs
	?f <- (cmd_many_people_posprs ?posprs1 ?posprs2 1)
	(item (name ?posprs1) (quantity ?manyPeople1))
	(item (name ?posprs2) (quantity ?manyPeople2))
	=>
	(retract ?f)
	;(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople1 " people " ?posprs1 ", The crowd have  " ?manyPeople2 " people " ?posprs2))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule many_people_gesture
	?f <- (cmd_many_people_gesture ?gesture 1)
	(item (name ?gesture) (quantity ?manyPeople))
	=>
	(retract ?f)
	;(bind ?manyPeople (random 5 10))
        (bind ?command (str-cat  "The crowd have " ?manyPeople " people " ?gesture))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;$crowdq = Was the $posprs person $gprsng?
(defrule many_people_was_two_gprsn_1
	?f <- (cmd_many_people_was ?posprs ?gprsn1 ?gprsn2 1)
	?f1 <- (crowd (name ?name)(pose ?pose&:(eq ?posprs ?pose)) (gender ?gender&:(or (eq ?gender ?gprsn1) (eq ?gender ?gprsn2))) )
	=>
	(retract ?f)
        (bind ?command (str-cat  "The " ?pose " person was a " ?gender))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule many_people_was_two_gprsn_3
	?f <- (cmd_many_people_was ?posprs ?gprsn1 ?gprsn2 1)
	(crowd (name ?name)(pose ?posprs) (gender ?gender&:(and (neq ?gender ?gprsn1) (neq ?gender ?gprsn2))) )
	(item (name ?posprs) (quantity ?q&:(> ?q 0)))
	=>
	(retract ?f)
        (bind ?command (str-cat  "There are no " ?posprs " person"))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule many_people_was_two_gpsrn_2
	?f <- (cmd_many_people_was ?posprs ?gprsn1 ?gprsn2 1)
	(item (name ?posprs) (quantity  0))
	=>
	(retract ?f)
	(bind ?command (str-cat "Neither, because there is zero " ?posprs " person"  ))
	(printout t ?command)	
)

;wdq = Tell me if the ($posprs | {gesture}) person was a $gprsn?
(defrule many_people_was_gprsn
	?f <- (cmd_many_people_was ?posprs ?gprsn 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 1 2))
	(if (= ?manyPeople 1)
	    	then (bind ?wasPeople (str-cat "Yes, the person " ?posprs " was a"))
	    	else (bind ?wasPeople (str-cat "No, the person " ?posprs " was not a"))
	)
        (bind ?command (str-cat ?wasPeople " " ?gprsn))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

(defrule many_people_was_gesture
	?f <- (cmd_many_people_was_gesture ?gesture ?gprsn 1)
	=>
	(retract ?f)
	(bind ?manyPeople (random 1 2))
	(if (= ?manyPeople 1)
	    	then (bind ?wasPeople (str-cat "Yes, the person " ?gesture " was a"))
	    	else (bind ?wasPeople (str-cat "No, the person " ?gesture " was not a"))
	)
        (bind ?command (str-cat ?wasPeople " " ?gprsn))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;$crowdq = Tell me how many people were wearing $color
(defrule many_people_color
	?f <- (cmd_many_people_color ?color 1)
	(item (name gender) (quantity ?q))
	=>
	(retract ?f)
	(bind ?manyPeople (random 0 ?q))
        (bind ?command (str-cat  "There were " ?manyPeople " people wearing " ?color))
        ;(assert (send-blackboard ACT-PLN query_result ?command 1 4))
        (printout t ?command)
)

;;;;insert quantity of gestures

;; insert 
(defrule set_gesture_q
	?f <- (cmd_set_gesture_q ?gesture ?q 1)
	?f1 <- (item (type Gesture) (name ?gesture))
	=>
	(retract ?f)
	(printout t "Se inserto " ?q " en " ?gesture)
	(modify ?f1 (quantity ?q))
)

;;;;;;; insert quantity genders
(defrule set_gender_q
	?f <- (cmd_set_gender_q ?gender ?q 1)
	?f1 <- (item (type Gender)(name ?gender))
	=>
	(retract ?f)
	(printout t "Se inserto " ?q " en " ?gender)
	(modify ?f1 (quantity ?q))

)

;;;;; insert quentity of poses
(defrule set_pose_q
	?f <- (cmd_set_pose_q ?pose ?q 1)
	?f1 <- (item (type Pose)(name ?pose))
	=>
	(retract ?f)
	(printout t "Se inserto " ?q " en " ?pose)
	(modify ?f1 (quantity ?q))
)

;;;;;insert total quantity
(defrule set_total_q
	?f <- (cmd_set_total_q ?total 1)
	?f1 <- (item (name gender))
	=> 
	(retract ?f)
	(printout t "Se inserto " ?total)
	(modify ?f1 (quantity ?total))
)

(defrule set_new_person_of_crowd
	?f <- (cmd_set_prsn ?name ?pose ?gender 1)
	=>
	(retract ?f)
	(printout t "Insert person " ?name " " ?pose " " ?gender)
	(assert (crowd (name ?name) (pose ?pose) (gender ?gender) (gesture nil)))
)
