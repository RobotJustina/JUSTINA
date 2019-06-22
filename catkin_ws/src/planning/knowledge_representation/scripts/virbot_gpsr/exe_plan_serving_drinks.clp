;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	10/Enero/2019			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	Serving Drinks			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; for find objects on the bar or another place

(defrule exe-plan-find-objects-on-location 
	(plan (name ?name) (number ?num-pln) (status active) (actions find-objects-on-location ?place) (duration ?t))
	=>
	(bind ?command (str-cat "" ?place "" ))
        (assert (send-blackboard ACT-PLN objects_on_location ?command ?t 4))
)

(defrule exe-plan-identified-objects
	?f <- (received ?sender command objects_on_location ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions find-objects-on-location ?place))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule identified-object 
	?f <- (update-object-on-location ?obj 1)
	?f1 <- (item (name ?obj))
	=>
	(retract ?f)
	(modify ?f1 (status on_the_bar))
)

(defrule on-the-bar-object-state
	?f <- (status-object-on-location ?obj 1)
	?f1 <- (item (name ?obj) (status ?status))
	=>
	(retract ?f)
	(printout t ?status)
)

;;;;;;;;;;;;;;;;

(defrule exe-plan-offer-drink
	(plan (name ?name) (number ?num-pln) (status active) (actions offer_drink ?eatdrink) (actions_num_params ?orders) (duration ?t))
	=>
	(bind ?command (str-cat "offer-drink"))
	(assert (send-blackboard ACT-PLN offer_drink ?command ?t 4))

)

(defrule exe-plan-offered-drink
	?f <- (received ?sender command offer_drink ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink ?eatdrink) (actions_num_params ?orders))
	?f2 <- (num_order ?n&:(< ?n (+ ?orders 1)))
	?f3 <- (item (name offer))
	?f4 <- (item (name people))
	?f5 <- (drink_o ?obj ?person ?num&:(eq (+ ?num 1) ?n))
	=>
	(retract ?f ?f5)
	(modify ?f1 (status accomplished))
	(modify ?f3 (status offered))
	(modify ?f4 (status offer));;;change last_offer for offer if you can ask many times for drinks
	(assert (drink_order ?obj ?person ?num))
)

(defrule exe-plan-offered-last-drink
	?f <- (received ?sender command offer_drink ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink ?eatdrink) (actions_num_params ?orders))
	?f2 <- (num_order ?n&:(eq ?n (+ ?orders 1)))
	?f3 <- (item (name offer))
	?f4 <- (item (name people))
	?f5 <- (drink_o ?obj ?person ?orders)
	=>
	(retract ?f ?f5)
	(modify ?f1 (status accomplished))
	(modify ?f3 (status offered))
	(modify ?f4 (status last_offer))
	(assert (drink_order ?obj ?person ?orders))
)

(defrule exe-plan-no-offered-drink
	?f <- (received ?sender command offer_drink ?param 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink ?eatdrink) (actions_num_params ?orders))
	?f2 <- (item (name offer))
	?f3 <- (item (name people))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status no_offered))
	(modify ?f3 (status no_offer))
)

(defrule give-drink-to-person
	?f <- (give-drink-to-person ?obj ?person)
	?f2 <- (num_order ?num)
	?f3 <- (item (name people))
	=>
	(retract ?f ?f2)
	(assert (drink_o ?obj ?person ?num))
	(assert (num_order (+ ?num 1)))
	(modify ?f3 (image ?person))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; train person
(defrule exe-plan-train-person
	(plan (name ?name) (number ?num-pln) (status active) (actions train_person) (duration ?t))
	(item (name people) (image ?person))
	=>
	(bind ?command(str-cat "" ?person ""))
	(assert (send-blackboard ACT-PLN train_person ?command ?t 4))
)

(defrule exe-plan-train-person-v2 
	(plan (name ?name) (number ?num-pln) (status active) (actions train_person ?person) (duration ?t))
	=>
	(bind ?command(str-cat "" ?person ""))
	(assert (send-blackboard ACT-PLN train_person ?command ?t 4))
)

(defrule exe-plan-trained-person
	?f <- (received ?sender command train_person ?person 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions train_person))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-trained-person-v2 
	?f <- (received ?sender command train_person ?person 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions train_person ?person))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;get order

(defrule exe-plan-get-order
	(plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place) (actions_num_params ?num_obj ?orders) (duration ?t))
	=>
	(assert (object_counter 0))
	(assert (order_counter 0))
	;(bind ?command(str-cat "get-order" ?obj ""))
	;(assert (send-blackboard ACT-PLN get-order ?command ?t 4))
)

(defrule exe-plan-get-order-every-object
	(plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place) (actions_num_params ?num_obj ?orders) (duration ?t))
	?f <- (object_counter ?n&:(< ?n ?num_obj))
	?f1 <- (order ?ord)
	?f2 <- (drink_order ?obj ?person ?num)
	?f3 <- (order_counter ?oc)
	=>
	(retract ?f ?f1 ?f2 ?f3)
	(bind ?command(str-cat "" ?ord "" ?obj "!" ))
	(assert (object_counter (+ ?n 1)))
	(assert (order ?command))
	(assert (deliver_order ?obj ?person ?num))
	(assert (order_counter (+ ?oc 1)))
)

(defrule exe-plan-get-order-last-object
	(plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place) (actions_num_params ?num_obj ?orders)(duration ?t))
	?f <- (object_counter ?num_obj)
	?f1 <- (order ?ord)
	;?f2 <- (drink_order ?obj ?person ?num)
	?f3 <- (order_counter ?oc&:(neq ?oc ?orders))
	=>
	(retract ?f ?f1)
	(assert (order !))
	(assert (d_order ?ord))
	(bind ?command (str-cat "" ?ord " " ?place))
	;(assert (deliver_order ?obj ?person ?num))
	(assert (send-blackboard ACT-PLN get_order ?command ?t 4))
)

(defrule exe-plan-get-order-last-order
	(plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place) (actions_num_params ?num_obj ?orders) (duration ?t))
	?f <- (order_counter ?orders)
	?f1 <- (object_counter ?n)
	?f2 <- (order ?ord)
	;?f3 <- (drink_order ?obj ?person ?num)
	?f3 <- (item (name people))
	=>
	(retract ?f ?f1 ?f2)
	(assert (order !))
	(assert (d_order ?ord))
	(bind ?command (str-cat "" ?ord " " ?place))
	(modify ?f3 (status deliver_order))
	;(assert (deliver_order ?obj ?person ?num))
	(assert (send-blackboard ACT-PLN get_order ?command ?t 4))
)

(defrule exe-plan-geted-order
	?f <- (received ?sender command get_order ?ord ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place)(actions_num_params ?num_obj ?orders)) 
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;deliver order

(defrule exe-plan-deliver-order
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions deliver_order ?place)(duration ?t))
	?f1 <- (d_order ?ord)
	=>
	(bind ?command(str-cat "" ?ord " " ?place ""))
	(assert (send-blackboard ACT-PLN deliver_order ?command ?t 4))
)

(defrule exe-plan-delivered-order
	?f <- (received ?sender command deliver_order ?ord ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions deliver_order ?place))
	?f2 <- (d_order ?o)
	=>
	(retract ?f ?f2)
	(modify ?f1 (status accomplished))
)

(defrule get-person 
	?f <- (get_person ?obj)
	?f1 <- (deliver_order ?obj ?person ?num)
	?f2 <- (set_object_arm ?obj ?arm)
	=>
	(retract ?f)
	(printout t ?person)
)

(defrule get-arm
	?f <-(get_arm ?obj)
	?f1 <- (set_object_arm ?obj ?arm)
	=>
	(retract ?f)
	(printout t ?arm)
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
