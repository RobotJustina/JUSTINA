
;;;;;;;;;;;;;;;;;;;;;;;;;;;;; reglas para subplanes ;;;;;;;;;;;;;;;;;;;;


(defrule exe-plan-ask-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?obj ?place)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone))
        =>
        (bind ?command (str-cat  "" ?obj " " ?place ""))
        (assert (send-blackboard ACT-PLN ask_for ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
)

(defrule exe-plan-asked-actuator_furniture
        ?f <-  (received ?sender command ask_for ?object ?zone 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object ?zone))
	(item (name ?zone) (type Furniture))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f1 (zone ?zone))
        ;(retract ?f4)
)

(defrule exe-plan-asked-actuator_room
        ?f <-  (received ?sender command ask_for ?object ?zone 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions ask_for ?object ?zone))
	(item (name ?zone) (type Room))
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
        ;?f2 <- (item (name ?obj2) (possession ?zone))
        =>
        (bind ?command (str-cat  "" ?zone ""))
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
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(modify ?f5 (status open))
        ;(retract ?f4)
)

(defrule exe-plan-no-stated-flase
        ?f <-  (received ?sender command status_object ?obj2 ?status 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions attend ?object))
        ?f3 <- (item (name robot))
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
        ?f3 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?question ?question_task)(duration ?t))
        (item (name ?question))
                ?f2 <- (item (name robot));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        =>
        (bind ?command (str-cat  "" ?question " " ?question_task ""))
        (assert (send-blackboard ACT-PLN answer ?command ?t 4))
        ;(waitsec 1) 
        ;(assert (wait plan ?name ?num-pln ?t))
        ;(modify ?f2 (zone frontexit));;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
)


(defrule exe-plan-question-ready
        ?f <-  (received ?sender command answer ?resp ?task 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?person ?task))
        ?f3 <- (item (name robot))
        ?f4 <- (item (name ?resp))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f4 (status ask))
)

(defrule exe-plan-question-no-ready
        ?f <-  (received ?sender command answer ?resp ?task 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions answer_question ?person ?task))
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

;;;;;;;;;;;;;;;;;;;;;;;;;;find specific person          +;;;;;;;;;;;;;;;;;;;;;;;;;;
               
 (defrule exe-plan-find-specific-person                
         (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?spc ?person)(duration ?t))           
       ?f1 <- (item (name ?person))            
        =>            
        (bind ?command (str-cat "" ?spc " " ?person ""))              
        (assert (send-blackboard ACT-PLN find_object ?command ?t 4))          
)             
              
(defrule exe-plan-found-specific-person               
        ?f <-  (received ?sender command find_object ?spc ?person 1)          
      ?f1 <- (item (name ?person))            
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?spc ?person))         
                      
        =>            
        (retract ?f)          
        (modify ?f2 (status accomplished))            
      (modify ?f1 (status went))                      
)             
              
(defrule exe-plan-no-found-specific-person            
        ?f <-  (received ?sender command find_object ?spc ?person 0)          
        ?f1 <- (item (name ?person))          
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?spc ?person))             
        =>            
        (retract ?f)          
        ;;(modify ?f2 (status active))
	;;;las siguientes dos lineas es solo para pasar al siguiente estado en el simulador
	(modify ?f2 (status accomplished))  ;; modificar esta parte del codigo
	(modify ?f1 (status went));;modificar esta parte del codigo         
)             

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;; speech any
(defrule exe-plan-speech-anything
	(plan (name ?name) (number ?num-pln) (status active) (actions speech-anything ?speech) (duration ?t))
	=>
	(bind ?command (str-cat "" ?speech ""))
	(assert (send-blackboard ACT-PLN spg_say ?command ?t 4))
)

(defrule exe-plan-speeched-anything
	?f <- (received ?sender command spg_say ?spc 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech-anything ?speech))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-no-speeched-anything
	?f <- (received ?sender command spg_say ?spc 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions speech-anything ?speech))
	=>
	(retract ?f)
	(modify ?f1 (status active))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;; confirmation

(defrule exe-plan-task-confirmation
	(plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf) (duration ?t))
	=>
	(bind ?command (str-cat "" ?conf ""))
	(assert (send-blackboard ACT-PLN cmd_task_conf ?command ?t 4))
)

(defrule exe-plan-task-confirmated
	?f <- (received ?sender command cmd_task_conf conf 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(assert (confirmation true))
)

(defrule exe-plan-task-no-confirmated
	?f <- (received ?sender command cmd_task_conf conf 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions confirmation ?conf))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(assert (confirmation false))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;

(defrule exe-plan-task-make-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?obj ?status) (actions_num_params ?ini ?end))
	?f1 <- (confirmation true)
	=>
	(retract ?f1)
	(modify ?f (status accomplished))
)

(defrule exe-plan-task-no-make-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?obj ?status) (actions_num_params ?ini ?end&:(< ?ini ?end)))
	?f1 <- (plan (name ?name) (number ?ini))
	;;?f1 <- (state (name ?plan) (status active) (number ?n))
	(confirmation false)
	=>
	(retract ?f1)
	(modify ?f (actions_num_params (+ ?ini 1) ?end))
)

(defrule exe-plan-task-no-make-last-task
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?obj ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini))
	?f2 <- (plan (name ?name) (number ?num&:(eq ?num (+ ?ini 1))))
	?f3 <- (confirmation false)
	=>
	(retract ?f1 ?f3)
	(modify ?f (status unaccomplished))
	(modify ?f2 (status active))
)

(defrule exe-plan-task-no-make-last-task-two
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions make_task ?name ?obj ?status) (actions_num_params ?ini ?ini))
	?f1 <- (plan (name ?name) (number ?ini))
	?f2 <- (item (name ?obj))
	?f4 <- (confirmation false)
	=>
	(retract ?f1 ?f4)
	(modify ?f (status accomplished))
	(modify ?f2 (status ?status))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; align with point

(defrule exe-plan-align-with-point
	(plan (name ?name) (number ?num-pln) (status active) (actions align_with_point ?obj ?ori_frame ?dest_frame) (duration ?t))
	(item (name ?obj) (pose ?x ?y ?z))
	=>
	(bind ?command (str-cat "" ?x " " ?y " " ?z " " ?ori_frame " " ?dest_frame ""))
	(assert (send-blackboard ACT-PLN cmd_align_point ?command ?t 4))
)

(defrule exe-plan-aligned-with-point
	?f <- (received ?sender command cmd_align_point ?x ?y ?z ?ori_frame ?dest_frame 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions align_with_point ?obj ?ori_frame ?dest_frame))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

(defrule exe-plan-no-aligned-with-point
	?f <- (received ?sender command cmd_align_point ?x ?y ?z ?ori_frame ?dest_frame 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions align_with_point ?obj ?ori_frame ?dest_frame))
	=>
	(retract ?f)
	(modify ?f1 (status active))
)

;;;;;;;;;;;;;;;;;;;;;; finish move of cube on top of cube ;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;; and cube on top of cubestable       ;;;;;;;;;;;;


(defrule exe-plan-pile-cube-on-top-cube
	?f <- (plan (name ?name) (number ?num-pln)(status active)(actions pile ?block1 ?block2)(duration ?t))
	?f1 <- (item (name ?block1))
	?stack-1 <- (stack ?block1 $?rest1)
	?stack-2 <- (stack ?block2 $?rest2)
	=>
	(retract ?stack-1 ?stack-2)
	(assert (stack $?rest1))
	(assert (stack ?block1 ?block2 $?rest2))
	;(assert (attempt (move ?block1) (on-top-of ?block2)(number 8 )))
	(modify ?f (status accomplished))
	(modify ?f1 (status on-top ?block2))
)

(defrule exe-plan-pile-cube-on-top-table
	?f <- (plan (name ?name) (number ?num-pln)(status active)(actions pile ?block1)(duration ?t))
	?f1 <- (item (name ?block1))
	?stack-1 <- (stack ?block1 $?rest1)
	=>
	(retract ?stack-1)
	(assert (stack $?rest1))
	(assert (stack ?block1))
	;(assert (attempt (move ?block1) (on-top-of cubestable)(number 7 ) ))
	(modify ?f (status accomplished))	
	(modify ?f1 (status on-top cubestable))
)

(defrule exe-plan-pile-dont-move
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions pile ?block1 ?block2 upgrade_state) (duration ?t))
	?f1 <- (item (name ?block1))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (status on-top ?block2))
)

(defrule upgrade-final-move-block
	?f <- (condition-block ?block1 ?block2)
	?f1 <- (item (name ?block1) (status on-top ?block2))
	?f2 <- (plan (name ?name) (number ?num-pln) (status active) (actions put_on_top ?block1 ?block2))
	=>
	(retract ?f)
	(modify ?f1 (status on-top))
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;; finish move simul;;;;;;;;;;;;;;;;;

(defrule exe-plan-pile-simul-cube-on-top-cube
	?f <- (plan (name ?name) (number ?num-pln)(status active)(actions pile_simul ?block1 ?block2)(duration ?t))
	?f1 <- (item (name ?block1))
	?stack-1 <- (stack ?block1 $?rest1)
	?stack-2 <- (stack ?block2 $?rest2)
	=>
	(retract ?stack-1 ?stack-2)
	(assert (stack $?rest1))
	(assert (stack ?block1 ?block2 $?rest2))
	(modify ?f (status accomplished))
	(modify ?f1 (attributes on-top ?block2))
)

(defrule exe-plan-pile-simul-cube-on-top-table
	?f <- (plan (name ?name) (number ?num-pln)(status active)(actions pile_simul ?block1)(duration ?t))
	?f1 <- (item (name ?block1))
	?stack-1 <- (stack ?block1 $?rest1)
	=>
	(retract ?stack-1)
	(assert (stack $?rest1))
	(assert (stack ?block1))
	(modify ?f (status accomplished))	
	(modify ?f1 (attributes on-top cubestable))
)

(defrule exe-plan-pile-simul-dont-move
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions pile_simul ?block1 ?block2 upgrade_state)(duration ?t))
	?f1 <- (item (name ?block1))
	=>
	(modify ?f (status accomplished))
	(modify ?f1 (attributes on-top ?block2))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;; verify arm 

(defrule verify_arm_disponible
        ?f <- (Arm (name ?arm) (status verify) (grasp nil))
        ?f1 <- (item (name ?object)(status finded))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
        =>
        (modify ?f (status ready)(grasp ?object))
        ;(modify ?f1 (status finded))
        (modify ?f2 (status accomplished))
)


(defrule verify_right_arm_no_disponible
        ?f <- (Arm (name right) (status verify) (grasp ?object1&:(neq ?object1 nil)))
        ?f3 <- (Arm (name left) (grasp ?x&:(eq ?x nil)))
        ?f1 <- (item (name ?object2) (status finded))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object2))
        =>
        (modify ?f (status ready))
        (modify ?f3 (status ready)(grasp ?object2))
        (modify ?f2 (status accomplished))
        ;(modify ?f1 (status finded))
)


(defrule verify_left_arm_no_disponible
        ?f <- (Arm (name left) (status verify) (grasp ?object1&:(neq ?object1 nil)))
        ?f3 <- (Arm (name right)(grasp ?x&:(eq ?x nil)))
        ?f1 <- (item (name ?object2)(status finded))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object2))
        =>
        (modify ?f (status ready))
        (modify ?f3 (status ready)(grasp ?object2))
        (modify ?f2 (status accomplished))
        ;(modify ?f1 (status finded))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule delate-empty-stack
	?f <- (stack)
	=>
	(retract ?f)
)

;;;;;;;;;;;;;;;;;;;;
