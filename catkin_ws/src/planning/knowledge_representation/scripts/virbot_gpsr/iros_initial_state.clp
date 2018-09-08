;************************************************
;*						*
;*	Initial state 				*
;*						*
;************************************************

(deffacts init-skts-blackboard
        (address BLACKBOARD "localhost" )
        ;(address BLACKBOARD "148.205.199.80" )
        (port_out BLACKBOARD  2300)

	; Network definitions
	(open-network BLACKBOARD)
)


(deffacts Initial-state-objects-rooms-zones-actors

; Zones definitions
	;(present-place nil)


	

;;;;;;;;;;;IROS 2018 names

	;;; SNACKS
	;(item (type Objects) (name chips)(zone desk)(image chips)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 1) (size 4) (wide 4) (height 3)(color yellow) (quantity 1) (grasp 1))
	;(item (type Objects) (name senbei)(zone desk)(image senbei)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 2) (size 3) (wide 1) (height 1)(color brown) (quantity 1) (grasp 2))
	;(item (type Objects) (name pringles)(zone desk)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 3) (size 2) (wide 3)(height 2)(color red) (quantity 1) (grasp 31))
	;(item (type Objects) (name peanuts)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 1) (wide 2) (height 4)(color blue) (quantity 1) (grasp 6))
	(item (type Objects) (name coke)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 1) (wide 2) (height 4)(color blue) (quantity 1) (grasp 6))

	;;;;;Piles of objects and persons locations
	(item (type Pile) (name dato_3) (image coke) (num 3) (zone desk))
	(item (type Pile) (name dato_2) (image coke) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image coke) (num 1) (zone bed))

	;;;; Fameles
	;(item (type Objects) (name sophie)(zone living_room)(image sophie) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name jackie)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alex)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charlie)(zone living_room)(image charlie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name elizabeth)(zone living_room)(image elizabeth) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name francis)(zone living_room)(image francis) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jennifer)(zone living_room)(image jennifer) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name linda)(zone living_room)(image linda) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mary)(zone living_room)(image mary) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name patricia)(zone living_room)(image patricia) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robin)(zone living_room)(image robin) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name skyler)(zone living_room)(image skyler) (attributes pick)(pose -1.87 8.64 0.0))

	;;; Males
	(item (type Objects) (name james)(zone living_room)(image james) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name john)(zone living_room)(image john) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image michael) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robert)(zone kitchen)(image robert) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name william)(zone living_room)(image william) (attributes pick)(pose -1.87 8.64 0.0))
	
	;;;ROOM
	( item (type Room) (name bedroom) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	;( item (type Room) (name office) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 1))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 8) (quantitys 2))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 1))
	;( item (type Room) (name bathroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name dining_room)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

	;;;placment
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
        (item (type Furniture) (name side_table)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	
	(item (type Furniture) (name end_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name couch)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	
	(item (type Furniture) (name cupboard)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name storage_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name sink)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name dishwasher)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Rooms definitions
	

	( item (type Door) (name entrance)(possession corridor)(attributes no_visited)(room corridor) (pose -3.55 -3.0 0.0))
	( item (type Door) (name exit)(possession living_room)(attributes no_visited)(room living_room)(pose -1.87 8.64 0.0))


; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))
	( Arm (name right)(bandera false))
	( Arm (name left) (bandera true))

; Furniture definitions
	( item (type Furniture) (name cubestable) )
	
	;;;;;
	;(item (type Objects) (name table) (image table)( attributes no-pick brown)(pose -3.55 -3.0 0.0))
	(item (type Person) (name man) (image man)(zone living_room)(pose -1.87 8.64 0.0))
        (item (type Person) (name man_guide) (image man)(zone living_room)(pose -1.87 8.64 0.0))
	(item (type Person) (name man_call) (image man)(zone living_room)(pose -1.87 8.64 0.0))
        (item (type Person) (name person) (image man)(zone living_room)(pose -1.87 8.64 0.0))

;EEGPSR category II auxiliar facts
	(item (type Objects) (name offer) (image offer))
	(item (type Objects) (name people) (image people))

; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )
	( item (type Door) (name fridgedoor) (status closed) (possession fridge))
	( item (type Door) (name cupboarddoor) (status closed) (possession cupboard))
	( item (type Zone) (name tablezone) (status closed) (possession table))

	(item (type Furniture) (name table)(pose -1.55 4.03 0.0))
	
	(item (type question) (name question_1) (status no_ask) (possession table))
	(item (type question) (name question_2) (status no_ask) (possession table))
	(item (type question) (name question_3) (status no_ask) (possession table))
	(item (type Door) (name exitdoor) (status no_ask) (possession table))
	;(item (type Furniture) (name shelf) (status no_ask))
	( item (type Room) (name current_loc) (pose -3.55 -3.0 0.0))
	(item (type Door) (name arena) (status no_ask) (possession table))
        (item (type Speech) (name speech_1) (image i_am_ready_for_a_new_category_2_command))
        (item (type Speech) (name speech_2) (image i_finish_the_test))
	(item (type Speech) (name speech))
        (item (type Speech) (name join_dark_side) (image i_always_belonged_to_the_dark_side))

;;;;;; hecho para guardar el satus de la confirmacion
	(item (type Confirmation) (name conf) (status nil))
	(item (type Confirmation) (name incomplete) (status nil))
)

