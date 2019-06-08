;;;;;;;;;;;;;;;
;;;; EEGPSR 
;;;; Julio Cruz
;;;; 6-6-2019
;;;;;;;;;;;;;;

(defrule exe-plan-task-get-color-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?color ?object)(duration ?t))
	(item (type Color) (name ?color))
	=>
	(bind ?command (str-cat "color " ?color " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-task-get-abspos-object-2
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?pos ?object)(duration ?t))
	(item (type Abspos) (name ?pos))
	=>
	(bind ?command (str-cat "abspose " ?pos " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-task-get-property-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?prop ?object)(duration ?t))
	(item (type Property) (name ?prop))
	=>
	(bind ?command (str-cat "property " ?prop " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

