;********************************************************
;*							*
;*	virbot_blacboard_sockets.clp	 		*
;*							*
;*			University of Mexico	        *
;*			J. Savage			*
;*							*
;*			20-3-2010			*
;*							*
;********************************************************


;********************************************************
;* 	Sockets Rules				        *
;********************************************************

(defglobal ?*id-act-pln* = 0)


; It opens the network to send messages to the indicated system
(defrule open-network-output
	  (declare (salience 10000))
          ?fact <- (open-network ?system)
	  (not (simulate ?system))
	  (address ?system ?address)
	  (port_out ?system ?port_out)
        =>
          (retract ?fact)

	  ; it opens the network address to send messages
    	  (printout t crlf "Messages for the " ?system "  will be sent to " ?address " port " ?port_out crlf)

          (bind ?transmitter (CONDOR_open_network_conection ?address ?port_out 512 "w"))
	  ;(bind ?message (str-cat "Stablish conection with " ?system))
	  ;(assert (send-speech ?message))
    	  (assert (transmitter ?system ?transmitter))
)


;it sends the command to the destination 
(defrule send-command-blackboard
           ?fact1 <- (send-blackboard ?source ?command ?arguments)
	   (transmitter BLACKBOARD ?transmitter-id)
        =>
          (retract ?fact1)
          (bind ?command1 (str-cat  ?source " " ?command " \""  ?arguments "\" @" ?*id-act-pln* ))
          (bind ?status (CONDOR_send_data_client_network ?transmitter-id ?command1))
          (printout t "command sent" ?command1 crlf)
          (bind ?*id-act-pln* (+ ?*id-act-pln* 1))
)



(defrule subscribe-shared-variable
        ?f <-(item (name ?name)(shared false))
        =>
        (modify ?f (shared true))
        (bind ?var (str-cat "" ?name " subscribe=writeothers report=content"  ""))
        (assert (send-blackboard ACT-PLN subscribe_var ?var))
        (bind ?var (str-cat "" "string " ?name ""))
        (assert (send-blackboard ACT-PLN create_var ?var))
)



(defrule transform-network-myexplode-network
        (declare (salience 8100))
        ?fact <- (network ?sender ?command ?str-parameters $?data)
        =>
        (retract ?fact)
        (myexplode ?str-parameters)
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


; (received BLK read_var { string outside-door "status open" % write_any % content % MVN-P } 1)
(defrule transform-network-blackboard
	?f1 <- (rec ?sender read_var ?dummy ?dummy1 ?var ?value $?rest)
        =>
        (retract ?f1)
        (myexplode ?value)
        (assert (blackboard ?sender read_var ?var))
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


 
