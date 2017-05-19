;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;		DEFTEMPLATES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(deftemplate waiting
	(slot cmd (type STRING))
	(slot id (type INTEGER))
	(slot args (type STRING))
	(slot timeout
		(type INTEGER)
		(range 0 ?VARIABLE)
	)
	(slot attempts
		(type INTEGER)
		(range 1 ?VARIABLE)
	)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;		BB FUNCTIONS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(deffunction send-command
	; Receives: command, cmd_params and optionally
	;timeout and number of attempts in case it times out or fails
	(?cmd ?args $?settings)
	(bind ?timeout ?*defaultTimeout*)
	(bind ?attempts ?*defaultAttempts*)
	(switch (length$ $?settings)
		(case 1 then
			(bind ?timeout (nth$ 1 $?settings))
		)
		(case 2 then
			(bind ?timeout (nth$ 1 $?settings))
			(bind ?attempts (nth$ 2 $?settings))
		)
	)
	(bind ?id (python-call SendCommand ?cmd ?args))
	(if ( > ?timeout 0) then
		(setCmdTimer ?timeout ?cmd ?id)
		(bind ?test-cmd (str-cat ?cmd))
		(assert
			(waiting (cmd ?test-cmd) (id ?id) (args ?args) (timeout ?timeout) (attempts ?attempts) )
		)
	)
	(log-message INFO "Sent command: '" ?cmd "' - id: " ?id " - timeout: " ?timeout "ms - attempts: " ?attempts)
)

(deffunction send-response
	(?cmd ?id ?result ?params)
	(python-call SendResponse ?cmd ?id ?result ?params)
	(log-message INFO "Sent response: '" ?cmd "' - id: " ?id " - result: " ?result "params: " ?params)
	(halt)
)

(deffunction create-shared_var
	; Receives: type and name
	;type is one of: byte[] | int | int[] | long | long[] | 
	; double | double[] | string | matrix | RecognizedSpeech | var
	(?type ?name)
	(bind ?resp (python-call CreateSharedVar ?type ?name))
	(if (eq ?resp 1) then
		(log-message INFO "Created SharedVar: " ?name)
	else
		(log-message WARNING "SharedVar: '" ?name "' could NOT be created!")
	)
	(return ?resp)
)

(deffunction write-shared_var
	; Receives: type, name and data
	;type is one of: byte[] | int | int[] | long | long[] | 
	; double | double[] | string | matrix | RecognizedSpeech | var
	(?type ?name $?data)
	(bind ?resp (python-call WriteSharedVar ?type ?name $?data))
	(if (eq ?resp 1) then
		(log-message INFO "Written to SharedVar: '" ?name "' - " $?data)
	else
		(log-message WARNING "Could NOT write to SharedVar: '" ?name "'")
	)
	(return ?resp)
)

(deffunction subscribe_to-shared_var
	; Receives: name and optionally subscription type and report type
	; Subscription type is one of: creation | writemodule | writeothers | writeany
	; report type is one of: content | notify
	(?name $?options)
	(bind ?resp (python-call SubscribeToSharedVar ?name $?options) )
	(if (eq ?resp 1) then
		(log-message INFO "Subscribed to SharedVar: '" ?name "'")
	else
		(log-message WARNING "Could NOT subscribe to SharedVar: '" ?name "'")
	)
	(return ?resp)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;		RULES TO HANDLE BB (PYTHON) ASSERTS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;(defrule BB-waiting-timedout-without_attempts
	;?w <-(waiting (cmd ?cmd) (id ?id) (attempts 1) (timeout ?timeout&~0) )
	;?BB <-(BB_timer ?cmd ?id)
	;(not
		;(BB_received ?cmd ?id $?)
	;)
	;=>
	;(retract ?w ?BB)
	;(log-message WARNING "Command timedout w/o attempts: '" ?cmd "' - id: " ?id " - timeout: " ?timeout "ms")
;)

;(defrule BB-waiting-timedout-with_attempts
	;?w <-(waiting (cmd ?cmd) (id ?id) (args ?args) (attempts ?attempts&~1) (timeout ?timeout&~0) )
	;?BB <-(BB_timer ?cmd ?id)
	;(not
		;(BB_received ?cmd ?id $?)
	;)
	;=>
	;(retract ?BB)
	;(log-message WARNING "Command timedout w/ attempts: '" ?cmd "' - id: " ?id " - timeout: " ?timeout "ms - attempts: " ?attempts)
	;(bind ?id (python-call SendCommand ?cmd ?args))
	;(setCmdTimer ?timeout ?cmd ?id)
	;(bind ?attempts (- ?attempts 1))
	;(modify ?w (id ?id) (attempts ?attempts))
	;(log-message INFO "Sent command: '" ?cmd "' - id: " ?id " - timeout: " ?timeout "ms - attempts: " ?attempts)
;)

;(defrule BB-failed-with_attempts
	;?w <-(waiting (cmd ?cmd) (id ?id) (args ?args) (attempts ?attempts&~1) (timeout ?timeout&~0) )
	;?BB <-(BB_received ?cmd ?id 0 ?)
	;=>
	;(retract ?BB)
	;(log-message WARNING "Command failed w/ attempts: '" ?cmd "' - id: " ?id " - attempts: " ?attempts)
	;(bind ?id (python-call SendCommand ?cmd ?args))
	;(setCmdTimer ?timeout ?cmd ?id)
	;(modify ?w (id ?id) (attempts (- ?attempts 1)))
	;(log-message INFO "Sent command: '" ?cmd "' - id: " ?id " - timeout: " ?timeout "ms - attempts: " ?attempts)
;)

;(defrule BB-clear-timers
	;?t <-(BB_timer ?cmd ?id)
	;(not
		;(waiting (cmd ?cmd) (id ?id))
	;)
	;=>
	;(retract ?t)
	;(log-message INFO "Clearing timer for command: '" ?cmd "' - id: " ?id)
;)

;(defrule BB-clear_response
	;?BB <-(BB_received ?cmd ?id $?)
	;(not
		;(waiting (cmd ?cmd) (id ?id))
	;)
	;=>
	;(retract ?BB)
	;(log-message WARNING "Clearing unhandled response from command: '" ?cmd "' - id: " ?id)
;)

;(defrule BB-set_answer
	;?w <-(waiting (cmd ?cmd) (id ?id) (attempts ?attempts))
	;?BB <-(BB_received ?cmd ?id ?result ?params)
	;(test (or (eq ?result 1) (eq ?attempts 1)))
	;=>
	;(retract ?w ?BB)
	;(assert 
		;(BB_answer ?cmd ?result ?params)
	;)
	;(log-message INFO "Answer received: '" ?cmd "' - id: " ?id " - successful: " ?result " - response: " ?params)
;)

;	HANDLE SHARED VAR UPDATES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule BB-set_sv_update
	?BB <-(BB_sv_updated ?var $?values)
	=>
	(retract ?BB)
	(printout t "Shared var updated: " ?var crlf "value: " $?values crlf)
)

(defrule BB-unknown-command
	(declare (salience -10000))
	?BB <-(BB_cmd ?cmd ?id ?params)
	=>
	(retract ?BB)
	(send-response ?cmd ?id FALSE "unknown command")
	(log-message WARNING "Unhandled command received: " ?cmd)
	(halt)
)
