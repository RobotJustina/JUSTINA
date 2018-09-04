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

(defrule insert_person_description_man
	?f <- (cmd_person_description man ?hight ?age ?weight 1)
	?f1 <- (person_description ?sp)
	=>
	(retract ?f ?f1)
	(bind ?command(str-cat "" ?sp "he_is_a_" ?hight "_and_" ?age "_man,_furthermore_I_would_say_that_his_complexion_is_" ?weight))
	(assert (person_description ?command))
)

(defrule insert_person_description_woman
	?f <- (cmd_person_description woman ?hight ?age ?weight 1)
	?f1 <- (person_description ?sp)
	=>
	(retract ?f ?f1)
	(bind ?command(str-cat "" ?sp "she_is_a_" ?hight "_and_" ?age "_woman,_furthermore_I_would_say_that_her_complexion_is_" ?weight))
	(assert (person_description ?command))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
