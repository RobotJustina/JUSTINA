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
	(init_pile set_params)
	

;;;;;;;;;;;IROS 2018 names

	;;; SNACKS
	;(item (type Objects) (name chips)(zone desk)(image chips)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 1) (size 4) (wide 4) (height 3)(color yellow) (quantity 1) (grasp 1))
	;(item (type Objects) (name senbei)(zone desk)(image senbei)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 2) (size 3) (wide 1) (height 1)(color brown) (quantity 1) (grasp 2))
	;(item (type Objects) (name pringles)(zone desk)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 3) (size 2) (wide 3)(height 2)(color red) (quantity 1) (grasp 31))
	;(item (type Objects) (name peanuts)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 1) (wide 2) (height 4)(color blue) (quantity 1) (grasp 6))
	(item (type Objects) (name coke)(zone kitchen_counter)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name water)(zone kitchen_counter)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name juice)(zone kitchen_counter)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bedroom))
	(item (type Objects) (name apple)(zone kitchen_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name lemon)(zone kitchen_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name rice)(zone kitchen_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name pringles)(zone kitchen_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name kleenex)(zone bathroom_drawer)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bathroom))
	(item (type Objects) (name sponge)(zone bathroom_drawer)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bathroom))
	(item (type Objects) (name soap)(zone bathroom_drawer)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bathroom))
	(item (type Objects) (name whiteboard_cleaner)(zone bathroom_drawer)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bathroom))
	(item (type Objects) (name cup)(zone dining_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room dining_room))
	(item (type Objects) (name glass)(zone dining_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room dining_room))
	(item (type Objects) (name candle)(zone dining_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bedrioom))
	(item (type Objects) (name reading_glasses)(zone dining_table)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bedrioom))

	;;;;;Piles of objects and persons locations
	(item (type Pile) (name dato_3) (image coke) (num 3) (zone kitchen_counter))
	(item (type Pile) (name dato_2) (image coke) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image coke) (num 1) (zone dining_table))
	
	(item (type Pile) (name dato_3) (image water) (num 3) (zone kitchen_counter))
	(item (type Pile) (name dato_2) (image water) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image water) (num 1) (zone dining_table))
	
	(item (type Pile) (name dato_3) (image juice) (num 3) (zone kitchen_counter))
	(item (type Pile) (name dato_2) (image juice) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image juice) (num 1) (zone dining_table))
	
	(item (type Pile) (name dato_3) (image pringles) (num 3) (zone kitchen_table))
	(item (type Pile) (name dato_2) (image pringles) (num 2) (zone kitchen_counter))
	(item (type Pile) (name dato_1) (image pringles) (num 1) (zone coffee_table))

	(item (type Pile) (name dato_3) (image apple) (num 3) (zone kitchen_table))
	(item (type Pile) (name dato_2) (image apple) (num 2) (zone kitchen_counter))
	(item (type Pile) (name dato_1) (image apple) (num 1) (zone coffee_table))

	(item (type Pile) (name dato_3) (image lemon) (num 3) (zone kitchen_table))
	(item (type Pile) (name dato_2) (image lemon) (num 2) (zone kitchen_counter))
	(item (type Pile) (name dato_1) (image lemon) (num 1) (zone coffee_table))

	(item (type Pile) (name dato_3) (image rice) (num 3) (zone kitchen_table))
	(item (type Pile) (name dato_2) (image rice) (num 2) (zone kitchen_counter))
	(item (type Pile) (name dato_1) (image rice) (num 1) (zone coffee_table))

	(item (type Pile) (name dato_3) (image kleenex) (num 3) (zone bathroom_drawer))
	(item (type Pile) (name dato_2) (image kleenex) (num 2) (zone side_table_2))
	(item (type Pile) (name dato_1) (image kleenex) (num 1) (zone kitchen_counter))

	(item (type Pile) (name dato_3) (image sponge) (num 3) (zone bathroom_drawer))
	(item (type Pile) (name dato_2) (image sponge) (num 2) (zone side_table_2))
	(item (type Pile) (name dato_1) (image sponge) (num 1) (zone kitchen_counter))

	(item (type Pile) (name dato_3) (image soap) (num 3) (zone bathroom_drawer))
	(item (type Pile) (name dato_2) (image soap) (num 2) (zone side_table_2))
	(item (type Pile) (name dato_1) (image soap) (num 1) (zone kitchen_counter))

	(item (type Pile) (name dato_3) (image whiteboard_cleaner) (num 3) (zone bathroom_drawer))
	(item (type Pile) (name dato_2) (image whiteboard_cleaner) (num 2) (zone side_table_2))
	(item (type Pile) (name dato_1) (image whiteboard_cleaner) (num 1) (zone kitchen_counter))

	(item (type Pile) (name dato_3) (image cup) (num 3) (zone coffee_table))
	(item (type Pile) (name dato_2) (image cup) (num 2) (zone kitchen_table))
	(item (type Pile) (name dato_1) (image cup) (num 1) (zone dining_table))

	(item (type Pile) (name dato_3) (image glass) (num 3) (zone coffee_table))
	(item (type Pile) (name dato_2) (image glass) (num 2) (zone kitchen_table))
	(item (type Pile) (name dato_1) (image glass) (num 1) (zone dining_table))

	(item (type Pile) (name dato_3) (image candle) (num 3) (zone side_table_2))
	(item (type Pile) (name dato_2) (image candle) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image candle) (num 1) (zone dining_table))
	
	(item (type Pile) (name dato_3) (image reading_glasses) (num 3) (zone side_table_2))
	(item (type Pile) (name dato_2) (image reading_glasses) (num 2) (zone coffee_table))
	(item (type Pile) (name dato_1) (image reading_glasses) (num 1) (zone dining_table))
	
	;;; Person
	(item (type Objects) (name steve)(zone living_room)(image steve) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robert)(zone living_room)(image robert) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name luis)(zone living_room)(image luis) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name john)(zone living_room)(image john) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name james)(zone living_room)(image james) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alfred)(zone living_room)(image alfred) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name daniel)(zone living_room)(image daniel) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name paul)(zone living_room)(image paul) (attributes pick)(pose -1.87 8.64 0.0))

	(item (type Pile) (name steve_dato_3)(image steve)(num 3) (zone bedroom))
	(item (type Pile) (name steve_dato_2)(image steve)(num 2) (zone living_room))
	(item (type Pile) (name steve_dato_1)(image steve)(num 1) (zone kitchen))
	
	(item (type Pile) (name robert_dato_3)(image robert)(num 3) (zone bedroom))
	(item (type Pile) (name robert_dato_2)(image robert)(num 2) (zone living_room))
	(item (type Pile) (name robert_dato_1)(image robert)(num 1) (zone kitchen))
	
	(item (type Pile) (name luis_dato_3)(image luis)(num 3) (zone bedroom))
	(item (type Pile) (name luis_dato_2)(image luis)(num 2) (zone living_room))
	(item (type Pile) (name luis_dato_1)(image luis)(num 1) (zone kitchen))
	
	(item (type Pile) (name john_dato_3)(image john)(num 3) (zone bathroom))
	(item (type Pile) (name john_dato_2)(image john)(num 2) (zone bedroom))
	(item (type Pile) (name john_dato_1)(image john)(num 1) (zone dining_room))
	
	(item (type Pile) (name james_dato_3)(image james)(num 3) (zone bathroom))
	(item (type Pile) (name james_dato_2)(image james)(num 2) (zone bedroom))
	(item (type Pile) (name james_dato_1)(image james)(num 1) (zone dining_room))
	
	(item (type Pile) (name alfred_dato_3)(image alfred)(num 3) (zone bathroom))
	(item (type Pile) (name alfred_dato_2)(image alfred)(num 2) (zone bedroom))
	(item (type Pile) (name alfred_dato_1)(image alfred)(num 1) (zone dining_room))
	
	(item (type Pile) (name daniel_dato_3)(image daniel)(num 3) (zone kitchen))
	(item (type Pile) (name daniel_dato_2)(image daniel)(num 2) (zone living_room))
	(item (type Pile) (name daniel_dato_1)(image daniel)(num 1) (zone hallway))
	
	(item (type Pile) (name paul_dato_3)(image paul)(num 3) (zone kitchen))
	(item (type Pile) (name paul_dato_2)(image paul)(num 2) (zone living_room))
	(item (type Pile) (name paul_dato_1)(image paul)(num 1) (zone hallway))
	
	;;;ROOM
	( item (type Room) (name bedroom) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	;( item (type Room) (name office) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 1))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 8) (quantitys 2))
	( item (type Room) (name hallway)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name bathroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name dining_room)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name entrance_hall)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name hallway_outside) (pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

	;;;placment
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name closet)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name side_table_1)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name side_table_2)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	
	(item (type Furniture) (name bookshelf)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name tv_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name coffee_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name arm_chair_1)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name arm_chair_2)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name armchair_1)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name armchair_2)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	
	(item (type Furniture) (name kitchen_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name kitchen_counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	
	(item (type Furniture) (name trash_bin)(pose -3.55 -3.0 0.0)(possession corridor)(attributes no_visited)(room hallway))
	(item (type Furniture) (name plant)(pose -3.55 -3.0 0.0)(possession corridor)(attributes no_visited)(room hallway))

	(item (type Furniture) (name bathroom_drawer)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	
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
	( item (type Room) (name current_person) (pose -3.55 -3.0 0.0))
	(item (type Door) (name arena) (status no_ask) (possession table))
        (item (type Speech) (name speech_1) (image i_finish_the_test))
        (item (type Speech) (name speech_2) (image i_complete_the_tasks,_was_a_pleasure_to_assist_you,_goodbye))
	(item (type Speech) (name speech))
        (item (type Speech) (name join_dark_side) (image i_always_belonged_to_the_dark_side))

;;;;;; hecho para guardar el satus de la confirmacion
	(item (type Confirmation) (name conf) (status nil))
	(item (type Confirmation) (name incomplete) (status nil))
;;;;;;; objeto para guardar status de terminacion de un plan
	(item (type Objects) (name finish_objetive)(status nil))
	(item (type Objects) (name finish_clips) (status nil))
)

