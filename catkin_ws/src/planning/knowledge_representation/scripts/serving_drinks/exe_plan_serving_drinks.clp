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
        (assert (send-blackboard ACT-PLN objects-on-location ?command ?t 4))
)

(defrule exe-plan-identified-objects
	?f <- (received ?sender command objects-on-location ?param 1)
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

;;;;;;;;;;;;;;;;

(defrule exe-plan-offer-drink
	(plan (name ?name) (number ?num-pln) (status active) (actions offer_drink) (actions_num_params ?orders) (duration ?t))
	=>
	(bind ?command (str-cat "offer-drink"))
	(assert (send-blackboard ACT-PLN offer_drink ?command ?t 4))

)

(defrule exe-plan-offered-drink
	?f <- (received ?sender command offer_drink ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink) (actions_num_params ?orders))
	?f2 <- (num_order ?n&:(< ?n (+ ?orders 1)))
	?f3 <- (item (name offer))
	?f4 <- (item (name people))
	?f5 <- (drink_o ?obj ?person ?num&:(eq (+ ?num 1) ?n))
	=>
	(retract ?f ?f5)
	(modify ?f1 (status accomplished))
	(modify ?f3 (status offered))
	(modify ?f4 (status offer))
	(assert (drink_order ?obj ?person ?num))
)

(defrule exe-plan-offered-last-drink
	?f <- (received ?sender command offer_drink ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink) (actions_num_params ?orders))
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
	?f <- (received ?sender command offer_drink ?parma 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer_drink) (actions_num_params ?orders))
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

(defrule exe-plan-trained-person
	?f <- (received ?sender command train_person ?person 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions train_person))
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
	(bind ?command(str-cat "" ?ord "" ?obj "_" ))
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
	?f3 <- (order_counter ?oc)
	=>
	(retract ?f ?f1 ?f3)
	(assert (order _))
	;(assert (deliver_order ?obj ?person ?num))
	(assert (send-blackboard ACT-PLN get-order ?ord ?t 4))
)

(defrule exe-plan-get-order-last-order
	(plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place) (actions_num_params ?num_obj ?orders) (duration ?t))
	?f <- (order_counter ?orders)
	?f1 <- (object_counter ?n)
	?f2 <- (order ?ord)
	;?f3 <- (drink_order ?obj ?person ?num)
	=>
	(retract ?f ?f1 ?f2 ?f3)
	(assert (order _))
	;(assert (deliver_order ?obj ?person ?num))
	(assert (send-blackboard ACT-PLN get-order ?ord ?t 4))
)

(defrule exe-plan-geted-order
	?f <- (received ?sender command get-order ?ord 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_ordered_objects ?place)(actions_num_params ?num_obj ?orders)) 
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;deliver order

(defrule exe-plan-deliver-order
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions deliver-order)(duration ?t))
	=>
	(bind ?command(str-cat "deliver-order" ?obj))
	(assert (send-blackboard ACT-PLN deliver-order ?command ?t 4))
)

(defrule exe-plan-delivered-order
	?f <- (received ?sender command deliver-order ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions deliver-order))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
