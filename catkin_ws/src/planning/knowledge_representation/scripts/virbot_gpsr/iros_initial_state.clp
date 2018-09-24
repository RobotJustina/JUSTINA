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
	(item (type Objects) (name coke)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name banana)(zone sink)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))
	(item (type Objects) (name beer)(zone bed)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room bedroom))
	(item (type Objects) (name pringles)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(room kitchen))

	;;;;;Piles of objects and persons locations
	(item (type Pile) (name dato_3) (image coke) (num 3) (zone night_table))
	(item (type Pile) (name dato_2) (image coke) (num 2) (zone wardrobe))
	(item (type Pile) (name dato_1) (image coke) (num 1) (zone bed))
	
	(item (type Pile) (name dato_3) (image banana) (num 3) (zone dresser))
	(item (type Pile) (name dato_2) (image banana) (num 2) (zone armchair))
	(item (type Pile) (name dato_1) (image banana) (num 1) (zone drawer))
	
	(item (type Pile) (name dato_3) (image beer) (num 3) (zone desk))
	(item (type Pile) (name dato_2) (image beer) (num 2) (zone sideboard))
	(item (type Pile) (name dato_1) (image beer) (num 1) (zone cutlery_drawer))
	
	(item (type Pile) (name dato_3) (image pringles) (num 3) (zone dining_table))
	(item (type Pile) (name dato_2) (image pringles) (num 2) (zone chair))
	(item (type Pile) (name dato_1) (image pringles) (num 1) (zone baby_chair))

	;;;; Fameles
	;(item (type Objects) (name sophie)(zone living_room)(image sophie) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name jackie)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alex)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jamie)(zone living_room)(image charlie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name morgan)(zone living_room)(image elizabeth) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name taylor)(zone living_room)(image francis) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name tracy)(zone living_room)(image jennifer) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jordan)(zone living_room)(image linda) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name hayden)(zone living_room)(image mary) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name peyton)(zone living_room)(image patricia) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robin)(zone living_room)(image robin) (attributes pick)(pose -1.87 8.64 0.0))

	;;; Males
	(item (type Objects) (name james)(zone living_room)(image james) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image michael) (attributes pick)(pose -1.87 8.64 0.0))

	(item (type Pile) (name james_dato_3)(image james)(num 3) (zone bookshelf))
	(item (type Pile) (name james_dato_2)(image james)(num 2) (zone sofa))
	(item (type Pile) (name james_dato_1)(image james)(num 1) (zone coffee_table))
	
	(item (type Pile) (name alex_dato_3)(image alex)(num 3) (zone center_table))
	(item (type Pile) (name alex_dato_2)(image alex)(num 2) (zone bar))
	(item (type Pile) (name alex_dato_1)(image alex)(num 1) (zone fireplace))
	
	(item (type Pile) (name jamie_dato_3)(image jamie)(num 3) (zone tv_couch))
	(item (type Pile) (name jamie_dato_2)(image jamie)(num 2) (zone microwave))
	(item (type Pile) (name jamie_dato_1)(image jamie)(num 1) (zone cupboard))
	
	(item (type Pile) (name morgan_dato_3)(image morgan)(num 3) (zone counter))
	(item (type Pile) (name morgan_dato_2)(image morgan)(num 2) (zone cabinet))
	(item (type Pile) (name morgan_dato_1)(image morgan)(num 1) (zone sink))
	
	(item (type Pile) (name taylor_dato_3)(image taylor)(num 3) (zone stove))
	(item (type Pile) (name taylor_dato_2)(image taylor)(num 2) (zone fridge))
	(item (type Pile) (name taylor_dato_1)(image taylor)(num 1) (zone freezer))
	
	(item (type Pile) (name tracy_dato_3)(image tracy)(num 3) (zone washing_machine))
	(item (type Pile) (name tracy_dato_2)(image tracy)(num 2) (zone dishwasher))
	(item (type Pile) (name tracy_dato_1)(image tracy)(num 1) (zone cabinet))
	
	(item (type Pile) (name jordan_dato_3)(image jordan)(num 3) (zone bidet))
	(item (type Pile) (name jordan_dato_2)(image jordan)(num 2) (zone shower))
	(item (type Pile) (name jordan_dato_1)(image jordan)(num 1) (zone bathtub))
	
	(item (type Pile) (name hayden_dato_3)(image hayden)(num 3) (zone toilet))
	(item (type Pile) (name hayden_dato_2)(image hayden)(num 2) (zone towel_rail))
	(item (type Pile) (name hayden_dato_1)(image hayden)(num 1) (zone bathroom_s_cabinet))
	
	(item (type Pile) (name peyton_dato_3)(image peyton)(num 3) (zone washbasin))
	(item (type Pile) (name peyton_dato_2)(image peyton)(num 2) (zone sink))
	(item (type Pile) (name peyton_dato_1)(image peyton)(num 1) (zone bed))
	
	(item (type Pile) (name robin_dato_3)(image robin)(num 3) (zone desk))
	(item (type Pile) (name robin_dato_2)(image robin)(num 2) (zone sink))
	(item (type Pile) (name robin_dato_1)(image robin)(num 1) (zone bed))
	
	(item (type Pile) (name michael_dato_3)(image michael)(num 3) (zone desk))
	(item (type Pile) (name michael_dato_2)(image michael)(num 2) (zone sink))
	(item (type Pile) (name michael_dato_1)(image michael)(num 1) (zone bed))
	
	;;;ROOM
	( item (type Room) (name bedroom) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	;( item (type Room) (name office) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 1))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 8) (quantitys 2))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name bathroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name dining_room)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

	;;;placment
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name night_table)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name wardrobe)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name dresser)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name armchair)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
        (item (type Furniture) (name drawer)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	(item (type Furniture) (name sideboard)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	(item (type Furniture) (name cutlery_drawer)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	(item (type Furniture) (name chair)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	(item (type Furniture) (name baby_chair)(pose -3.55 -3.0 0.0)(possession dining_room)(attributes no_visited)(room dining_room))
	
	(item (type Furniture) (name bookshelf)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name tv_couch)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name coffee_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name center_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name bar)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name fireplace)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	
	(item (type Furniture) (name microwave)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name cupboard)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name cabinet)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name sink)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name stove)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name freezer)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name washing_machine)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name dishwasher)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	
	(item (type Furniture) (name cabinet_1)(pose -3.55 -3.0 0.0)(possession corridor)(attributes no_visited)(room corridor))

	(item (type Furniture) (name bidet)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name shower)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name bathtub)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name toilet)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name towel_rail)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name bathroom_s_cabinet)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	(item (type Furniture) (name washbasin)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room bathroom))
	

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
        (item (type Speech) (name speech_1) (image i_am_ready_for_a_new_command))
        (item (type Speech) (name speech_2) (image i_finish_the_test))
	(item (type Speech) (name speech))
        (item (type Speech) (name join_dark_side) (image i_always_belonged_to_the_dark_side))

;;;;;; hecho para guardar el satus de la confirmacion
	(item (type Confirmation) (name conf) (status nil))
	(item (type Confirmation) (name incomplete) (status nil))
;;;;;;; objeto para guardar status de terminacion de un plan
	(item (type Objects) (name finish_objetive)(status nil))
)

