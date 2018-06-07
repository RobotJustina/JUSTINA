;;;;;;;;;;;;;;
;;	Julio Cruz
;;	UNAM
;;;;;;;;;;;;;

(defrule insert_order
	?f <- (cmd_add_order ?eatdrink ?obj 1)
	?f1 <- (num_order ?num)
	?f2 <- (order ?sp)
	=>
	(retract ?f ?f1 ?f2)
	(bind ?command(str-cat "" ?sp ",_for_order_" ?num "_i_need_" ?obj ""))
	(assert (order ?command))
	(assert (num_order (+ 1 ?num)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
