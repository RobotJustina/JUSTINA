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


; Actors definitions
	;(robots-hands registration_form)
	;(present-actor robot)
	;(present-recepient You)
	;(present-master son)

; Objects definitions
	
	(item (type Objects) (name cranberry_juice)(zone kitchen_table)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coke)(zone kitchen_table)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name grape_juice)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name nescafe_latte)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	
	(item (type Objects) (name mexican_fritters)(zone dinner_table)(image crackers)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pocky)(zone dinner_table)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name soup)(zone dinner_table)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name soap)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))
	(item (type Objects) (name ajax)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))
	(item (type Objects) (name shoe_cleaner)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))

	(item (type Objects) (name choco_syrup)(zone kitchen_table)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coke)(zone kitchen_table)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coconut_milk)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name shampoo)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	
		(item (type Objects) (name soup)(zone table)(image soup)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coffe)(zone table)(image coffe)(attributes pick)(pose -3.55 -3.0 0.0))
	
	(item (type Objects) (name sugar)(zone table)(image pyramid)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name juice)(zone table)(image cube)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name milk)(zone table)(image cube)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cube_aux)(zone table)(image no_grap)(attributes pick)(pose -3.55 -3.0 0.0))	

	;;;;CUBES
	(item (type Objects) (name red_block) (zone table) (image cube) (attributes pick) (pose 0.0 0.0 0.0)(num 0.092))
	(item (type Objects) (name orange_block) (zone table) (image cube) (attributes pick) (pose 0.0 0.0 0.0)(num 0.092))
	(item (type Objects) (name blue_block) (zone table) (image cube) (attributes pick) (pose 0.0 0.0 0.0)(num 0.092))
	(item (type Objects) (name green_block) (zone table) (image cube) (attributes pick) (pose 0.0 0.0 0.0)(num 0.092))
	(item (type Objects) (name yellow_block) (zone table) (image pyramid) (attributes pick) (pose 0.0 0.0 0.0) (num 0.092))	

	(item (type Objects) (name person)(zone living_room)(image person) (attributes pick)(pose -0.5 0.0 0.0))
	(item (type Objects) (name susan)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mary_jane)(zone living_room)(image Beth) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name gabrielle)(zone living_room)(image Alfred) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name elsa)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))

	(item (type Objects) (name james)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robert)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name arthur)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mike)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name richi)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name noah)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))

	(item (type Objects) (name john)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name peter)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	

	(item (type Objects) (name world))	


; Rooms definitions
	
	( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))

	( item (type Door) (name entrance) (pose -3.55 -3.0 0.0))
	( item (type Door) (name exit)(pose -1.87 8.64 0.0))

	( item (type Room) (name inspection)(pose -1.87 8.64 0.0))
	
; Humans definitions
	;( Human (name Mother)(room bedroom)(pose 79.048340 76.107002 0.0))
	;( Human (name Father)(room kitchen)(pose 9.048340 6.107002 0.0))
	;( Human (name You)(room livingroom)(pose 9.048340 6.107002 0.0))


; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))
	( Arm (name right)(bandera false))
	( Arm (name left) (bandera true))

; Furniture definitions
	( item (type Furniture) (name cubestable) )

	(item (type Furniture) (name kitchen_table)(pose -3.55 -3.0 0.0))
	(item (type Furniture) (name stove)(pose -3.55 -3.0 0.0))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0))

	(item (type Furniture) (name dinner_table)(pose -3.55 -3.0 0.0))

	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0))
	(item (type Furniture) (name side_table)(pose -3.55 -3.0 0.0))
	
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0))
	(item (type Furniture) (name couch)(pose -3.55 -3.0 0.0))
	(item (type Furniture) (name coffe_table)(pose -3.55 -3.0 0.0))
	
	(item (type Furniture) (name open_table)(pose -3.55 -3.0 0.0))
	
	(item (type Furniture) (name cubestable)(pose -3.55 -3.0 0.0))	
	
	;;;;;
	;(item (type Objects) (name table) (image table)( attributes no-pick brown)(pose -3.55 -3.0 0.0))
	(item (type Person) (name man) (image man)(zone living_room)(pose -1.87 8.64 0.0))

; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )
	( item (type Door) (name fridgedoor) (status closed) (possession fridge))
	( item (type Door) (name cupboarddoor) (status closed) (possession cupboard))
	( item (type Zone) (name tablezone) (status closed) (possession table))

; Human-Robot-Interface
	;( item (type H-R-Interface) (name naturallanguage) (status nil) )
;;;;;;;;;new forniture for task "ask for"
	(item (type Furniture) (name cupboard)(pose -1.3 6.05 0.0))
	(item (type Furniture) (name fridge)(pose -4.07 6.64 0.0))
	(item (type Furniture) (name table)(pose -1.55 4.03 0.0))
	
	(item (type question) (name question) (status no_ask) (possession table))
	(item (type Door) (name exitdoor) (status no_ask) (possession table))
	(item (type Door) (name inspection) (status no_ask) (possession table))
	(item (type Furniture) (name shelf) (status no_ask))
	(item (type Room) (name current_loc) (pose -3.55 -3.0 0.0))
	
	(item (type Objects) (name stack)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name stack_exp)(pose -3.55 -3.0 0.0))
	(item (type Speech) (name speech_1)(image that_is_what_happened,_Now_I_make_a_new_plan_for_complete_the_command))
	(pile (name original))
	(pile (name simul))

	(item (type Objects) (name red_exp) (image red_block))	
	(item (type Objects) (name green_exp) (image green_block))	
	(item (type Objects) (name blue_exp) (image blue_block))
	(item (type Objects) (name blue_exp) (image yellow_block))

	(simul_moves 0)	

)


