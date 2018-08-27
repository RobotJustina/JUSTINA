;********************************************************
;*							*
;*	virbot_blacboard.clp		 		*
;*							*
;*			University of Mexico	        *
;*			J. Savage			*
;*							*
;*			9-5-2014			*
;*							*
;********************************************************


;********************************************************
;* 	Sockets Rules				        *
;********************************************************

(defglobal ?*id-act-pln* = 0)


; simulation of no answer from a command
(defrule delete-bb-none
	(declare (salience 1000))
	?f1 <- (BB_received ?cmd ?id ?dummy "None")
        =>
        (retract ?f1)
	;(assert (delete BB_timer ?id))
)


(defrule delete-bb-speak
	?f1 <- (BB_received "speak" $?)
        =>
        (retract ?f1)
)

(defrule transform-bb
	?f2 <- (waiting (cmd ?command) (id ?id))
	?f1 <- (BB_received ?command ?id ?status ?param)
        =>
        (retract ?f1 ?f2)
        (bind ?cmd (explode$ ?command))
        (bind $?prms (explode$ ?param))
        (assert (received blackboard command ?cmd $?prms ?status))
	(assert (delete BB_timer ?id))
        ;(printout t "Received " ?cmd " " ?param " " ?status " @" id crlf)
)


(defrule transform-bb-no-waiting
        ?f1 <- (BB_received ?command ?id ?status ?param)
        (not  (waiting (cmd ?command) (id ?id)))
        =>
        (retract ?f1)
        (bind ?cmd (explode$ ?command))
        (bind $?prms (explode$ ?param))
        (assert (received blackboard command ?cmd $?prms ?status))
        (assert (delete BB_timer ?id))
)



;it sends the command to the destination 
(defrule send-command-blackboard
           ?fact1 <- (send-blackboard ?source ?command ?arguments $?options)
        =>
          (retract ?fact1)
	  (send-command ?command ?arguments $?options)
          (printout t "Sent command " ?command " arguments " ?arguments " options " $?options crlf)
          ;(printout t ?command "" ?arguments"" crlf)
)



;(defrule create-subscribe-shared-variable
;	(declare (salience 100))
;       ?f <-(item (name ?name)(shared false))
;      =>
;        (modify ?f (shared true))
;	(create-shared_var string ?name)
;	(subscribe_to-shared_var ?name writeany content) 
;	(printout t "create and subscribe " ?name crlf)
;)



(defrule transform-network-myexplode-network
        (declare (salience 8100))
        ?fact <- (network ?sender ?command ?str-parameters $?data)
        =>
        (retract ?fact)
        ;(myexplode ?str-parameters)
        (assert (net ?sender ?command $?data))
)

(defrule new-network-myexplode-network
        (declare (salience 8000))
        ?fact <- (myexplode $?str-parameters)
        ?f <- (net ?sender ?command $?data)
        =>
        (retract ?f ?fact)
        (assert (rec ?sender ?command $?str-parameters $?data))
)



(defrule network-myexplode-blackboard
        ?fact <- (myexplode $?str-parameters)
        ?f <- (blackboard ?sender ?command ?var)
        =>
        (retract ?f ?fact)
        (assert (received ?sender ?command ?var $?str-parameters))
)



(defrule change-shared-variable-status
	?f1 <- (received ?sender read_var ?name status ?value)
	?f <-(item (name ?name))
	=>
	(retract ?f1)
	(modify ?f (status ?value))
)


(defrule change-shared-variable-zone
        ?f1 <- (received ?sender read_var ?name zone ?value)
        ?f <-(item (name ?name))
        =>
        (retract ?f1)
        (modify ?f (zone ?value))
)


(defrule change-command-variable-zone
        ?f1 <- (received ?sender command goto robot ?value ?x ?y ?z 1);;;;jc
        ?f <-(item (name robot))
        =>
        (retract ?f1)
        (modify ?f (zone ?value))
)


;fix this later
(defrule leave-same-variable-zone
        ?f1 <- (received ?sender command goto robot ?value ?x ?y ?z 0) ;;;jc
        ?f <-(item (name robot)(zone ?zone))
        (condition (conditional if) (arguments robot zone ?value)(true-state ?step1)(false-state ?step)(name-scheduled ?plan)(state-number ?step));;;
        =>
        (assert (cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to ?value)(name-scheduled ?plan)(state-number ?step)));;;;;
	
        (retract ?f1)
        (modify ?f (zone ?zone))
)

(defrule change-command-variable-status
        ?f1 <- (received ?sender command status_object ?sensor ?name ?value 1)
        ?f <-(item (name ?name))
        =>
        (retract ?f1)
        (modify ?f (status ?value))
)

 
(defrule leave-command-variable-status
        ?f1 <- (received ?sender command status_object ?sensor ?name ?value 0)
        ?f <-(item (name ?name)(status ?old))
        =>
        (retract ?f1)
        (modify ?f (status ?old))
)
 
