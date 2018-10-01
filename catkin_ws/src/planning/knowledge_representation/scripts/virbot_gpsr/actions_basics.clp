;********************************************************
;*                                                      *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      01/10/2018                      *
;*                                                      *
;********************************************************

(defrule task_finish_make_plan_clips
        ?f <- (task ?plan finish_clips ?param1 ?step)
        ?f1 <- (item (name ?param1))
        =>
        (retract ?f)
        (printout t "Finish plan generation" crlf)
        (assert (state (name ?plan) (number ?step)(duration 6000)))
        (assert (condition (conditional if) (arguments ?param1 status finished)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
        (assert (task pfinishclips ?param1 ?step))
        ;;;;;test reiniciar status del parametro
        (modify ?f1 (status nil))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule plan_finish_clips
        ?goal <- (objetive finish_clips ?name ?param ?step)
        =>
        (retract ?goal)
        (bind ?speech(str-cat "finish_plan"))
        (assert (plan (name ?name) (number 1) (actions send_clips_signal finish_clips) (duration 6000)))
        (assert (plan (name ?name) (number 2) (actions update_status ?param finished) (duration 6000)))
        (assert (finish-planner ?name 2))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe_finish-clips
        (state (name ?name) (number ?step) (status active) (duration ?time))
        (item (name ?robot) (zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (task pfinishclips ?param ?step)
        =>
        (retract ?f1)
        (assert (objetive finish_clips task_finish_clips ?param ?step))
)

;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-send-clips-signal
        (plan (name ?name) (number ?num-pln)(status active)(actions send_clips_signal ?param)(duration ?t))
        =>
        (bind ?command (str-cat "" ?param ""))
        (assert (send-blackboard ACT-PLN cmd_clips_signal ?command ?t 4))
	;(waitsec 1)
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-sended-clips-signal
        ?f <-  (received ?sender command cmd_clips_signal ?param 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions send_clips_signal ?param))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished));;;; modified for verify arm task		
)

(defrule exe-plan-no-send-clips-signal
        ?f <-  (received ?sender command find_object ?param 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions send_clips_signal ?param))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished)) ;performance for IROS
)

