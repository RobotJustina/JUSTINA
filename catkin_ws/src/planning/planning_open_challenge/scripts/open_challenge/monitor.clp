;********************************************************
;*                                                      *
;*      monitor.clp	                                *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage                    *
;*			Adrian Revueltas                *
;*                                                      *
;*                      2/4/14                          *
;*                                                      *
;********************************************************


(defrule wait-command
	(declare (salience -100))
	?f1 <- (BB_timer ?cmd ?id)
	?f2 <- (waiting (cmd ?cmd) (id ?id) (args ?args) (timeout ?timeout) (attempts ?att))
        =>
	(if ( > ?att 0) then
		;(retract ?f2 ?f1)
		(bind ?cmd (explode$ ?cmd))
		;(assert (send-blackboard ACT-PLN ?cmd ?args ?timeout (- ?att 1)))
	 else
		(retract ?f1)
		(assert (error cmd ?cmd ?id) )
	)
)


(defrule clean-wait-timer
	(declare (salience 100))
 	?f <- (delete BB_timer ?id)
 	?f1 <- (BB_timer ?cmd ?id)
	=>
	(retract ?f ?f1)
)


;fix this later, to check if the system is alive
(defrule error-command
        ?f1 <- (error cmd ?cmd ?id)
        ?f2 <- (waiting (cmd ?cmd) (id ?id) (args ?args) (timeout ?timeout) (attempts ?att))
        =>
        (retract ?f2 ?f1)
        (bind ?cmd (explode$ ?cmd))
        (assert (send-blackboard ACT-PLN ?cmd ?args ?timeout 4))
)


;(defrule wait-sec-state
	;;(declare (salience -100))
        ;?f1 <- (ret_waitsec ?num_msec)
        ;=>
	;(retract ?f1)
        ;(assert (waitsec state ?num_msec))
	;(assert (waitsec plan ?num_msec))
;)



;(defrule wait-second
	;(state (name ?name) (number ?num-state)(status active)(duration ?t))
        ;=>
        ;(waitsec 1)
	;(assert (wait state ?name ?num-state ?t))
;)

;(defrule clean-wait
	;(declare (salience 100))
        ;?f2 <- (wait state ?name ?num-state ?w)
        ;(state (name ?name) (number ?num-state)(status inactive)(duration ?t))
        ;=>
        ;(retract ?f2)
;)


;(defrule exe-plan-wait
	;?f1 <- (waitsec plan ?num_msec)
        ;?f2 <- (wait plan ?name ?num-pln ?w)
        ;(plan (name ?name) (number ?num-pln)(status active)(duration ?t))
        ;=>
        ;(retract ?f1 ?f2)
        ;(if ( > ?w 1) then
                ;(assert (wait plan ?name ?num-pln (- ?w 1)) )
                ;(waitsec 1)
         ;else
                ;(assert (error plan ?name ?num-pln) )
        ;)
;)






