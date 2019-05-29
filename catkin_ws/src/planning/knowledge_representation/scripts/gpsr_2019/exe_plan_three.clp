;********************************************************
;*                                                      *
;*      actions_three.clp                               *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      03/04/2019                      *
;*                                                      *
;********************************************************


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  get bag task
(defrule exe-plan-task-get-bag 
	(plan (name ?name) (number ?num-pln) (status active) (actions get_bag) (duration ?t))
	=>
	(bind ?command (str-cat "bag"))
	(assert (send-blackboard ACT-PLN cmd_get_bag ?command ?t 4))
)

(defrule exe-plan-task-geted-bag 
	?f <- (received ?sender command cmd_get_bag bag 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
        ?f2 <- (Arm (name right))
	?f3 <- (item (name bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status ready) (grasp bag))
	(modify ?f3 (status grabed))
)

(defrule exe-plan-task-no-geted-bag 
	?f <- (received ?sender command cmd_get_bag bag 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;; follow to the taxi, uber, cab
(defrule exe-plan-task-follow-task
	(plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place) (duration ?t))
	=>
	(bind ?command (str-cat "" ?place ""))
	(assert (send-blackboard ACT-PLN cmd_follow_to_taxi ?command ?t 4))
)

(defrule exe-plan-task-followed-to-taxi 
	?f <- (received ?sender command cmd_follow_to_taxi ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	?f2 <- (item (name man))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status followed))
)

(defrule exe-plan-no-followed-to-taxi
	?f <- (received ?sender command cmd_follow_to_taxi ?place 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;; get the abspos object
(defrule exe-plan-task-get-abspos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?pos)(duration ?t))
	(item (type Abspos) (name ?pos))
	=>
	(bind ?command (str-cat "abspose "  ?place " " ?pos ""))
	(assert (send-blackboard ACT-PLN find_object ?command ?t 4))
)

(defrule exe-plan-task-get-relpos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?pos ?object)(duration ?t))
	(item (type Relpos) (name ?pos))
	=>
	(bind ?command (str-cat "relpose "  ?place " " ?pos " " ?object ""))
	(assert (send-blackboard ACT-PLN find_object ?command ?t 4))
)

(defrule exe-plan-task-get-oprop-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?oprop ?category)(duration ?t))
	(item (type Property) (name ?oprop))
	=>
	(bind ?command (str-cat "for_grasp " ?oprop " " ?category ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-geted-pos-object 
        ?f <-  (received ?sender command ?find_object ?block1 ?x ?y ?z ?arm 1)
 	    ?f1 <- (item (name ?block1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-pos-object $?params))
	    ?f3 <- (Arm (name ?arm))
	?f4 <- (item (name object))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f3 (status ready) (grasp ?block1))
	(modify ?f1 (pose ?x ?y ?z) (status finded));;;; modified for verify arm task
	(modify ?f4 (image ?block1)(status finded))
)

(defrule exe-plan-no-geted-pos-object 
        ?f <-  (received ?sender command ?find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?block1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-pos-object $?params))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;;;;;;; grasp object

(defrule exe-plan-grasp-pos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions move ?actuator)(duration ?t))
	(item (name object) (image ?obj))
	(item (name ?obj) (pose ?x ?y ?z))
        (Arm (name ?arm) (status ready)(bandera ?id) (grasp ?obj))
	=>
        (bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
)

(defrule exe-plan-grasped-pos-object 
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator))
	?f3 <- (item (name robot));;;;;;;;;; T1 test for quit grasp object subtask
	?f4 <- (item (name object))
	?f5 <- (Arm (grasp ?object))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f3 (hands ?object));;;;; T1 test
	;(modify ?f1 (status grabed));;;;;; T1 test
	(modify ?f4 (status grabed))
	(modify ?f5 (grasp object))
        ;(retract ?f3)
)

(defrule exe-plan-no-grasped-pos-object
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator))
	?f3 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status nil) (grasp nil))
)


;;;;;;;;;;;;;;;; thre oprop objects
(defrule exe-plan-find-three-opos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions property_object ?oprop ?category three)(duration ?t))
	=>
        (bind ?command (str-cat "find_three_obj " ?oprop " " ?category ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-finded-three-oprop-object 
        ?f <-  (received ?sender command prop_obj ?type ?oprop ?category ?speech 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions property_object $?params))
	?f3 <- (item (name speech))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f3 (image ?speech))
)

(defrule exe-plan-no-finded-three-oprop-object 
        ?f <-  (received ?sender command ?find_object ?type ?oprop ?category ?speech 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions property_object $?params))
	?f3 <- (item (name speech))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (image ?speech))
)

;;;;;;;;;;;  introduce person to people
(defrule exe-plan-introduce-person 
	(plan (name ?name) (number ?num-pln) (status active) (actions introduce-person ?p ?person ?php ?place)(duration ?t))
	=>
        (bind ?command (str-cat "" ?p " " ?person " " ?php  " " ?place ""))
	(assert (send-blackboard ACT-PLN introduce_person ?command ?t 4))
)

(defrule exe-plan-introduced-person 
        ?f <-  (received ?sender command introduce_person ?p ?person ?php ?place 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions introduce-person $?params))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

(defrule exe-plan-no-introduced-person
        ?f <-  (received ?sender command introduce_person ?p ?person ?php ?place 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions introduce-person $?params))
	?f3 <- (item (name speech))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;;;;; make a question
(defrule exe-plan-make-a-question 
	(plan (name ?name) (number ?num-pln) (status active) (actions make_question ?type ?q ?conf)(duration ?t))
	=>
        (bind ?command (str-cat "" ?type " " ?q  " " ?conf ""))
	(assert (send-blackboard ACT-PLN make_question ?command ?t 4))
)

(defrule exe-plan-made_a_question
        ?f <-  (received ?sender command make_question ?t ?q ?conf 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions make_question $?params))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

(defrule exe-plan-no-made-a-question
        ?f <-  (received ?sender command make_question ?t ?q ?conf 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions make_question $?params))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
)
;;;;;;

(defrule exe-plan-guide-to-taxi 
	(plan (name ?name) (number ?num-pln) (status active) (actions guide_to_taxi ?person ?question)(duration ?t))
	=>
        (bind ?command (str-cat "" ?person " " ?question ""))
	(assert (send-blackboard ACT-PLN guide_to_taxi ?command ?t 4))
)

(defrule exe-plan-guided_to_taxi
        ?f <-  (received ?sender command guide_to_taxi ?person ?question 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions guide_to_taxi $?params))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

(defrule exe-plan-no-guided-to-taxi
        ?f <-  (received ?sender command guide_to_taxi ?person ?question 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions guide_to_taxi $?params))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;; clean up
(defrule exe-plan-clean-up 
	(plan (name ?name) (number ?num-pln) (status active) (actions clean_up ?room)(duration ?t))
	=>
        (bind ?command (str-cat "" ?room ""))
	(assert (send-blackboard ACT-PLN clean_up ?command ?t 4))
)

(defrule exe-plan-cleaned-up 
        ?f <-  (received ?sender command clean_up ?room 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions clean_up $?params))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

(defrule exe-plan-no-cleaned-up
        ?f <-  (received ?sender command clean_up ?room 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions clean_up $?params))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
)

;;;;;;;;;;;;;; take out the garbage
(defrule exe-plan-take-out-garbage 
	(plan (name ?name) (number ?num-pln) (status active) (actions take_out_garbage ?garbage)(duration ?t))
	=>
        (bind ?command (str-cat "" ?garbage ""))
	(assert (send-blackboard ACT-PLN take_out_garbage ?command ?t 4))
)

(defrule exe-plan-taked_out_garbage 
        ?f <-  (received ?sender command take_out_garbage ?garbage 1)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions take_out_garbage $?params))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

(defrule exe-plan-no-taked-up-garbage
        ?f <-  (received ?sender command take_out_garbage ?garbage 0)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions take_out_garbage $?params))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;
