;********************************************************
;*							*
;*	schedule_cubes.clp				*
;*							*
;*							*
;*			University of Mexico		*
;*			Jesus Savage			*
;*			Adrian Revueltas		*
;*			Mauricio Matamoros		*
;*							*
;*			22/01/14			*
;*							*
;********************************************************
;
; Scene:
;	1) The robot waits for the door to be open
;	2) The robot enters into the room
;	3) The robot goes to the table where the objects are
;	4) The robot puts the cubes in the asked order 
;	5) The robot leaves the arena.


(deffacts scheduled_cubes

	(name-scheduled cubes 1 5)


	; 				STATE 1
	; The robot waits until the door is open  
	; DURATION
	(state (name cubes) (number 1)(duration 6000)(status active))
	; INPUTS
	(condition (conditional if) (arguments outsidedoor status closed)(true-state 1)(false-state 2)(name-scheduled cubes)(state-number 1))
	; ACTIONS
	(cd-task (cd speak) (actor robot)(message "I'm checking if the door is open")(name-scheduled cubes)(state-number 1))
       	(cd-task (cd attend) (actor robot)(obj outsidedoor)(from sensors)(to status)(name-scheduled cubes)(state-number 1))



	; 				STATE 2
	; The robot enters in the arena
	; DURATION
	(state (name cubes) (number 2)(duration 6000))
	; INPUTS
	(condition (conditional if) (arguments robot zone frontentrance)(true-state 3)(false-state 2)(name-scheduled cubes)(state-number 2))
	; ACTIONS
	(cd-task (cd ptrans) (actor robot)(obj robot)(from frontexit)(to frontentrance)(name-scheduled cubes)(state-number 2))




	; 				STATE 3
	; The robot goes to the table where the cubes are
	; DURATION
	(state (name cubes) (number 3)(duration 6000))
	; INPUTS
	(condition (conditional if) (arguments robot zone cubestable)(true-state 4)(false-state 3)(name-scheduled cubes)(state-number 3))
	; ACTIONS
	(cd-task (cd ptrans) (actor robot)(obj robot)(from frontentrance)(to cubestable)(name-scheduled cubes)(state-number 3))





	; 				STATE 4
	; The robot speaks that is ready to accept spoken commands
	; and waits for it and executes it 
	; DURATION
	(state (name cubes) (number 4)(duration 3000))
	; INPUTS
	(condition (conditional if) (arguments blockC status on-top-of blockF)(true-state 5)(false-state 4)(name-scheduled cubes)(state-number 4))
	; ACTIONS
	(cd-task (cd speak) (actor robot)(message "I will acomodate the cubes")(name-scheduled cubes)(state-number 4))
	(cd-task (cd ptrans) (actor robot)(obj blockC)(to blockF)(name-scheduled cubes)(state-number 4))




	; 				STATE 5
	; The robot leaves the arena 
	; DURATION
	(state (name cubes) (number 5)(duration 6000))
	; INPUTS
	(condition (conditional if) (arguments robot zone outside)(true-state 6)(false-state 5)(name-scheduled cubes)(state-number 5))
	; ACTIONS
        (cd-task (cd speak) (actor robot)(message "I'm going to the outside door")(name-scheduled cubes)(state-number 5) )
        (cd-task (cd ptrans) (actor robot)(obj robot)(from nil)(to outside)(name-scheduled cubes)(state-number 5))


	;                               STATE 6
        ; The robot says the exit message
        ; DURATION
        (state (name cubes) (number 6)(duration 6000))
        ; ACTIONS
        (cd-task (cd speak) (actor robot)(message "I'm in the outside door")(name-scheduled cubes)(state-number 6) )


)

