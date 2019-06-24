;;;;;;;;;;;;;;;
;;;; EEGPSR 
;;;; Julio Cruz
;;;; 6-6-2019
;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;

(defrule exe-plan-task-get-color-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?color ?object)(duration ?t))
	(item (type Color) (name ?color))
	=>
	(bind ?command (str-cat "color " ?color " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-task-get-abspos-object-2
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?pos ?object)(duration ?t))
	(item (type Abspos) (name ?pos))
	=>
	(bind ?command (str-cat "abspose " ?pos " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

(defrule exe-plan-task-get-property-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?prop ?object)(duration ?t))
	(item (type Property) (name ?prop))
	=>
	(bind ?command (str-cat "property " ?prop " " ?object ""))
	(assert (send-blackboard ACT-PLN prop_obj ?command ?t 4))
)

;;;;;;;;;;;;;;;;;;


(defrule exe-plan-task-get-rpose-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-rpose-object ?place ?rpose ?category)(duration ?t))
	;(item (type Property) (name ?oprop))
	=>
	(bind ?command (str-cat "rpose " ?place " "?rpose " " ?category ""))
	(assert (send-blackboard ACT-PLN rpose_obj ?command ?t 4))
)

(defrule exe-plan-geted-rpose-object 
        ?f <-  (received ?sender command ?find_object ?object ?x ?y ?z ?arm 1)
 	?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-rpose-object $?params))
	?f3 <- (Arm (name ?arm))
	?f4 <- (item (name object))
	?f5 <- (item (name robot))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f3 (status ready) (grasp object))
	;(modify ?f1 (pose ?x ?y ?z) (status finded));;;; modified for verify arm task
	(modify ?f5 (hands ?object));;;;; T1 test
	(modify ?f4 (image ?object) (status grabed))
)

(defrule exe-plan-no-geted-rpose-object 
        ?f <-  (received ?sender command ?find_object ?object ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-rpose-object $?params))
	?f3 <- (Arm (name ?arm))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished)) ;performance for IROS
	(modify ?f3 (status nil) (grasp nil))
)

;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-pourin-obj 
	(plan (name ?name) (number ?num-pln) (status active) (actions pourin_object ?pourable ?canpourin ?person)(duration ?t))
	;(item (type Property) (name ?oprop))
	=>
	(bind ?command (str-cat "" ?pourable " " ?canpourin " " ?person ""))
	(assert (send-blackboard ACT-PLN pourin_obj ?command ?t 4))
)

(defrule exe-plan-pourined-obj 
        ?f <-  (received ?sender command pourin_obj ?pourable ?canpourin ?person 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions pourin_object $?params))
        =>
        (retract ?f)
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-pourined-obj 
        ?f <-  (received ?sender command pourin_obj ?pourable ?canpourin ?person 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions pourin_object $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-task-storage-obj 
	(plan (name ?name) (number ?num-pln) (status active) (actions storage_object ?object ?loc ?storage ?sp_obj)(duration ?t))
	;(item (type Property) (name ?oprop))
	=>
	(bind ?command (str-cat "" ?object " " ?loc " " ?storage " " ?sp_obj ""))
	(assert (send-blackboard ACT-PLN storage_obj ?command ?t 4))
)

(defrule exe-plan-task-storage-category-v2
	(plan (name ?name) (number ?num-pln) (status active) (actions storage_object)(duration ?t))
	;(item (type Property) (name ?oprop))
	?f <- (set_category_state ?category ?loc ?storage ?sp_obj)
	=>
	(retract ?f)
	(bind ?command (str-cat "" ?category " " ?loc " " ?storage " " ?sp_obj ""))
	(assert (send-blackboard ACT-PLN storage_obj ?command ?t 4))
)

(defrule exe-plan-task-storage-category 
	(plan (name ?name) (number ?num-pln) (status active) (actions storage_object ?storage ?sp_obj)(duration ?t))
	(item (name ?obj) (status grabed))
	(Arm (name ?arm) (bandera ?flag) (grasp ?obj))
	=>
	(bind ?command (str-cat "" ?obj " " ?flag " " ?storage " " ?sp_obj ""))
	(assert (send-blackboard ACT-PLN storage_obj ?command ?t 4))
	(assert (reset_arm storage_object ?arm))
)

(defrule exe-plan-task-storage-category-neg 
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions storage_object ?storage ?sp)(duration ?t))
	(not (item (name ?obj) (status grabed)))
	=>
	(modify ?f (status accomplished))
)

(defrule exe-plan-storage-obj 
        ?f <-  (received ?sender command storage_obj ?obj ?loc ?storage ?sp_obj 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions storage_object $?params))
	?f2 <- (item (name ?obj))
	(not (reset_arm storage_object ?arm))
        =>
        (retract ?f)
        (modify ?f2 (status nil))
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-storage-obj 
        ?f <-  (received ?sender command storage_obj ?obj ?loc ?storage ?sp_obj 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions storage_object $?params))
	?f2 <- (item (name ?obj))
	(not (reset_arm storage_object ?arm))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
        (modify ?f2 (status nil))
)

(defrule exe-plan-storage-obj-reset-arm 
        ?f <-  (received ?sender command storage_obj ?obj ?loc ?storage ?sp_obj 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions storage_object $?params))
	?f2 <- (item (name ?obj))
	?f3 <- (reset_arm storage_obj ?arm)
	?f4 <- (Arm (name ?arm))
        =>
        (retract ?f ?f3)
        (modify ?f2 (status nil))
        (modify ?f1 (status accomplished))
	(modify ?f4 (status nil) (grasp nil))
)

(defrule exe-plan-no-storage-obj-reset-arm 
        ?f <-  (received ?sender command storage_obj ?obj ?loc ?storage ?sp_obj 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions storage_object $?params))
	?f2 <- (item (name ?obj))
	?f3 <- (reset_arm storage_obj ?arm)
	?f4 <- (Arm (name ?arm))
        =>
        (retract ?f ?f3)
	(modify ?f1 (status accomplished)) ;performance for IROS
        (modify ?f2 (status nil))
	(modify ?f4 (status nil) (grasp nil))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule exe-plan-set-category-state
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions set_category_state ?obj ?loc))
	=>
	(assert (set_category_state ?obj ?loc))
	(modify ?f (status accomplished))
)

(defrule exe-plan-get-category-state
	?f <- (plan (name ?name) (number ?num-pln) (status active) (actions get_category_state ?storage ?sp_obj))
	?f1 <- (set_category_state ?obj ?loc)
	=>
	(retract ?f1)
	(assert (set_category_state ?obj ?loc ?storage ?sp_obj))
	(modify ?f (status accomplished))
)
;;;;;;;;;;;;;;;;;;

(defrule exe-plan-task-get-object-description 
	(plan (name ?name) (number ?num-pln) (status active) (actions obj_desc ?obj ?place)(duration ?t))
	=>
	(bind ?command (str-cat "" ?obj " " ?place ""))
	(assert (send-blackboard ACT-PLN obj_desc ?command ?t 4))
)

(defrule exe-plan-geted-object-description 
        ?f <-  (received ?sender command obj_desc ?obj ?place 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions obj_desc $?params))
        =>
        (retract ?f)
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-geted-object-desription
        ?f <-  (received ?sender command obj_desc ?obj ?place 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions obj_desc $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-retrieve-object 
	(plan (name ?name) (number ?num-pln) (status active) (actions retrieve_object ?cat ?place ?sp)(duration ?t))
	=>
	(bind ?command (str-cat "" ?cat " " ?place " " ?sp ""))
	(assert (send-blackboard ACT-PLN retrieve_object ?command ?t 4))
)

(defrule exe-plan-retrieved-object 
        ?f <-  (received ?sender command retrieve_object ?cat ?place ?sp 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions retrieve_object $?params))
        =>
        (retract ?f)
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-retrieve_object 
        ?f <-  (received ?sender command retrieve_object ?cat ?place ?sp 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions retrieve_object $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-interact_with_door 
	(plan (name ?name) (number ?num-pln) (status active) (actions interact_with_door ?door ?place ?action)(duration ?t))
	=>
	(bind ?command (str-cat "" ?door " " ?place " " ?action ""))
	(assert (send-blackboard ACT-PLN interact_with_door ?command ?t 4))
)

(defrule exe-plan-interacted-with-door  
        ?f <-  (received ?sender command interact_with_door ?door ?place ?action 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions interact_with_door $?params))
        =>
        (retract ?f)
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-interacted-with-door 
        ?f <-  (received ?sender command interact_with_door ?door ?place ?action 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions interact_with_door $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-set_tableware 
	(plan (name ?name) (number ?num-pln) (status active) (actions set_tableware ?tableware ?place)(duration ?t))
	=>
	(bind ?command (str-cat "" ?tableware " " ?place ""))
	(assert (send-blackboard ACT-PLN set_tableware ?command ?t 4))
)

(defrule exe-plan-seted-tableware 
        ?f <-  (received ?sender command set_tableware ?tableware ?place 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions set_tableware $?params))
	?f2 <- (item (name ?tableware))
	?f3 <- (item (name ?place))
        =>
        (retract ?f)
	(modify ?f2 (status set_tableware))
	(modify ?f3 (status set_tableware))
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-seted-tableware 
        ?f <-  (received ?sender command set_tableware ?tableware ?place 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions set_tableware $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished)) ;performance for IROS
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-set_cutlery 
	(plan (name ?name) (number ?num-pln) (status active) (actions set_cutlery ?cutlery ?pos ?tableware ?place)(duration ?t))
	?f <- (item (name ?cutlery) (zone ?zone))
	=>
	(bind ?command (str-cat "" ?cutlery " " ?zone " " ?pos " " ?tableware " " ?place ""))
	(assert (send-blackboard ACT-PLN set_cutlery ?command ?t 4))
)

(defrule exe-plan-seted-cutlery 
        ?f <-  (received ?sender command set_cutlery ?cutlery ?zone ?pos ?tableware ?place 1)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions set_cutlery $?params))
        =>
        (retract ?f)
        (modify ?f1 (status accomplished))
)

(defrule exe-plan-no-seted-cutlery 
        ?f <-  (received ?sender command set_cutlery ?cutlery ?zone ?pos ?tableware ?place 0)
        ?f1 <- (plan (name ?name) (number ?num-pln)(status active)(actions set_cutlery $?params))
        =>
        (retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe-plan-task-reset_arms
	?f <-(plan (name ?name) (number ?num-pln) (status active) (actions rev_arms ?arm)(duration ?t))
	?f1 <- (Arm (name ?arm) (status ?st))
	?f2 <- (item (name ?arm))
	=>
	(modify ?f (status accomplished))
	(modify ?f2 (status ?st))
)

(defrule exe-plan-task-reset_one_arm
	?f <-(plan (name ?name) (number ?num-pln) (status active) (actions rev_arms ?arm1 ?arm2)(duration ?t))
	?f1 <- (Arm (name ?arm1) (status ready))
	?f2 <- (Arm (name ?arm2) (status ready))
	?f3 <- (item (name right))
	=>
	(modify ?f (status accomplished))
	(modify ?f3 (status ready))
)

(defrule exe-plan-task-reset_one_arm_1
	?f <-(plan (name ?name) (number ?num-pln) (status active) (actions rev_arms ?arm1 ?arm2)(duration ?t))
	?f1 <- (Arm (name ?arm1) (status ?st1&:(neq ?st1 ready)))
	?f3 <- (item (name right))
	=>
	(modify ?f (status accomplished))
	(modify ?f3 (status nil))
)

(defrule exe-plan-task-reset_one_arm_2
	?f <-(plan (name ?name) (number ?num-pln) (status active) (actions rev_arms ?arm1 ?arm2)(duration ?t))
	?f1 <- (Arm (name ?arm2) (status ?st1&:(neq ?st1 ready)))
	?f3 <- (item (name right))
	=>
	(modify ?f (status accomplished))
	(modify ?f3 (status nil))
)
;;;;;;;;;;;;;;;;
