(deffacts test-facts
	(test_fact 1)		;triggers the call to the remote function
)

(defrule test-rule-command
	?f <-(test_fact 1)
	=>
	(retract ?f)
	(create-shared_var "string" "test_clips_var")
	(write-shared_var "string" "test_clips_var" "Updating var from CLIPS")
	(subscribe_to-shared_var "test_shared_var" )
	(send-command "cmd_one" test "params cmd one")
)

(defrule test-rule-answer
	?BB <-(BB_answer "cmd_one" ?result ?params)
	;?BB <-(BB_answer "cmd_one" 1 ?params)		;to catch only successful responses
	;?BB <-(BB_answer "cmd_one" 0 ?params)		;to catch only unsuccessful responses
	; other facts would probably be necessary
	;to disable other rules expecting an answer for the same command
	=>
	(retract ?BB)
	(log-message INFO "Answer for \"cmd_one\" received: " ?result " - " ?params)
	(send-command "cmd_two" test "params cmd two" 2000 2)
	(log-message INFO "Sending command: \"cmd_two\"...")
)

(defrule test-rule-answer_2
	;Should never fire, as this command will sleep and request will timeout before an answer is received.
	?BB <-(BB_answer "cmd_two" ?result ?params)
	=>
	(retract ?BB)
	(printout t "Final cmd result: " ?result crlf)
	(if (eq ?result 0) then
		(log-message WARNING "Second command failed!")
	)
)

(defrule respond_command-rule
	?BB <-(BB_cmd "clips_cmd" ?id ?params)
	=>
	(retract ?BB)
	(printout t "Received comand: 'clips_cmd' : " ?params crlf)
	(send-response "clips_cmd" ?id TRUE "clips command response")
)
