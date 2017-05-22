
;;;;;;;;;;;;;;;;;;;;;;;;;;;;; reglas para subplanes ;;;;;;;;;;;;;;;;;;;;


(defrule exe-plan-ask-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?obj)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone))
        =>
        (bind ?command (str-cat  "" ?obj ""))
        (assert (send-blackboard ACT-PLN ask_for ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-asked-actuator
        ?f <-  (received ?sender command ask_for ?object ?zone 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(modify ?f1 (zone ?zone))
        ;(retract ?f4)
)

(defrule exe-plan-no-asked-actuator
        ?f <-  (received ?sender command ask_for ?object ?zone 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;regla para moverse hacia un objeto

(defrule exe-plan-go-actuator
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(statusTwo active)(actions go_to ?object)(duration ?t))
        ?f1 <- (item (name ?object) (zone ?zone))
        (item (name ?zone)(pose ?x ?y ?z))
                ?f2 <- (item (name robot))
        =>
        (bind ?command (str-cat  "" ?object " " ?zone " " ?x " " ?y " " ?z ""))
        (assert (send-blackboard ACT-PLN goto ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
                (modify ?f2 (zone frontexit))
        (modify ?f3 (statusTwo inactive))
)

(
defrule exe-plan-went-actuator
        ?f <-  (received ?sender command goto ?object ?zone ?x ?y ?z 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
)

(defrule exe-plan-no-go-actuator
        ?f <-  (received ?sender command goto ?object ?zone ?x ?y ?z 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
        (modify ?f2 (statusTwo active))
)

;;;;;regla para status de puerta
(defrule exe-plan-status-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object)(duration ?t))
        ?f1 <- (item (name ?object) (zone ?zone))
        ?f2 <- (item (name ?obj2) (possession ?zone))
        =>
        (bind ?command (str-cat  "" ?obj2 ""))
        (assert (send-blackboard ACT-PLN status_object ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(
defrule exe-plan-stated-actuator
        ?f <-  (received ?sender command status_object ?obj2 open 1)
        ?f1 <- (item (name ?object) (zone ?zone))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        ?f5<- (item (name ?obj2) (possession ?zone))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f5 (status open))
        ;(retract ?f4)
)

(defrule exe-plan-no-stated-flase
        ?f <-  (received ?sender command status_object ?obj2 ?status 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
        
)

(defrule exe-plan-no-stated-true
        ?f <-  (received ?sender command status_object ?obj2 closed 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
        
)

;;;;;;;;;;;;;;;;;; reglas para moverse hacia un lugar

(defrule exe-plan-go-place
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(statusTwo active)(actions go_to_place ?place)(duration ?t))
        (item (name ?place)(pose ?x ?y ?z))
                ?f2 <- (item (name robot));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        =>
        (bind ?command (str-cat  "" ?place " " ?place " " ?x" " ?y " " ?z ""))
        (assert (send-blackboard ACT-PLN goto ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
                (modify ?f2 (zone frontexit));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (modify ?f3 (statusTwo inactive))
)

(
defrule exe-plan-went-place
        ?f <-  (received ?sender command goto ?zone ?place ?x ?y ?z 1)
        ;?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to_place ?place))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f4)
)

(defrule exe-plan-no-go-place
        ?f <-  (received ?sender command goto ?zone ?place ?x ?y ?z 0)
        ;?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to_place ?place))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (statusTwo active))
        
)


;;;;;;;;;;;;;;;;;;; reglas para moverse hacia una persona

(defrule exe-plan-go-person
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(statusTwo active)(actions go_to_person ?person)(duration ?t))
        (item (name ?person)(pose ?x ?y ?z))
                ?f2 <- (item (name robot));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        =>
        (bind ?command (str-cat  "" ?person " " ?person " " ?x" " ?y " " ?z ""))
        (assert (send-blackboard ACT-PLN goto ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
        (modify ?f2 (zone frontexit));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        (modify ?f3 (statusTwo inactive))
)

(
defrule exe-plan-went-person
        ?f <-  (received ?sender command goto ?zone ?person ?x ?y ?z 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to_person ?person))
        ?f3 <- (item (name robot))
        ?f4 <- (item (name ?person))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f4 (status went))
)

(defrule exe-plan-no-go-person
        ?f <-  (received ?sender command goto ?zone ?person ?x ?y ?z 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go_to_person ?person))
        ?f3 <- (item (name robot))
        =>
        (retract ?f)
        (modify ?f2 (statusTwo active))
        
)

;;;;;;;;;;;;;;;;;;; wait for instruction (question, name, team name, introduce itself)

(defrule exe-answer-question
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?question)(duration ?t))
        (item (name ?question))
                ?f2 <- (item (name robot));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        =>
        (bind ?command (str-cat  "" ?question))
        (assert (send-blackboard ACT-PLN answer ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
        ;(modify ?f2 (zone frontexit));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
)


(defrule exe-plan-question-ready
        ?f <-  (received ?sender command answer ?resp 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?person))
        ?f3 <- (item (name robot))
        ?f4 <- (item (name ?resp))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f4 (status ask))
)

(defrule exe-plan-question-no-ready
        ?f <-  (received ?sender command answer ?resp 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?person))
        ?f3 <- (item (name robot))
        ?f4 <- (item (name ?resp))
        =>
        (retract ?f)
        (modify ?f4 (name ?resp))
)

;;;;;;;;;;;;;;;follow man

(defrule exe-plan-find-object-man
        (plan (name ?name) (number ?num-pln)(status active)(actions find-object-man ?obj ?place)(duration ?t))
 	?f1 <- (item (name ?obj))
        =>
        (bind ?command (str-cat "" ?obj " " ?place ""))
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))
)

(defrule exe-plan-found-object-man
        ?f <-  (received ?sender command find_object ?man ?place 1)
 	?f1 <- (item (name ?man)(type Person))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object-man ?man ?place))
	
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f1 (status followed))	
)

(defrule exe-plan-no-found-object-man
        ?f <-  (received ?sender command find_object ?man ?place 0)
        ?f1 <- (item (name ?man)(type Person))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object-man ?man ?place))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;
