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
	(plan (name ?name) (number ?num-pln) (status active) (actions find-objects-on-location) (duration ?t))
	=>
	(bind ?command (str-cat "identify-objects" ))
        (assert (send-blackboard ACT-PLN objects-on-location ?command ?t 4))
)

(defrule exe-plan-identified-objects
	?f <- (received ?sender command objects-on-location ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions identify-objects))
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
	(plan (name ?name) (number ?num-pln) (status active) (actions offer-drink) (duration ?t))
	=>
	(bind ?command (str-cat "offer-drink"))
	(assert (send-blackboard ACT-PLN offer-drink ?command ?t 4))

)

(defrule exe-plan-offered-drink
	?f <- (received ?sender command offer-drink ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions offer-drink))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule give-drink-to-person
	?f <- (give-drink-to-person ?obj ?person)
	?f2 <- (num_order ?num)
	=>
	(retract ?f ?f2)
	(assert (drink_order ?obj ?person ?num))
	(assert (num_order (+ ?num 1)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;get order

(defrule exe-plan-get-order
	(plan (name ?name) (number ?num-pln) (status active) (actions get-orders) (duration ?t))
	?f <- (oreder-1 ?obj ?person)
	=>
	(bind ?command(str-cat "get-order" ?obj ""))
	(assert (send-blackboard ACT-PLN get-order ?command ?t 4))
)

(defrule exe-plan-geted-order
	?f <- (received ?sender command get-order ?param 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get-orders))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished));;;; falta agregar un ciclo para agarrar las 3 ordenes
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
