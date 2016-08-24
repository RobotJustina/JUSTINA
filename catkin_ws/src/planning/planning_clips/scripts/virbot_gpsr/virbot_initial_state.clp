
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
	
	;(item (type Objects) (name cranberry_juice)(zone kitchen_table)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name coke)(zone kitchen_table)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name grape_juice)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name nescafe_latte)(zone kitchen_table)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	
	;(item (type Objects) (name mexican_fritters)(zone dinner_table)(image crackers)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name pocky)(zone dinner_table)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0))
	;(item (type Objects) (name soup)(zone dinner_table)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0))

	;(item (type Objects) (name soap)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))
	;(item (type Objects) (name ajax)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))
	;(item (type Objects) (name shoe_cleaner)(zone side_table)(image sponge)(attributes pick)(pose -1.0 -3.0 0.0))

	

	(item (type Objects) (name person)(zone living_room)(image person) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name susan)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name mary_jane)(zone living_room)(image Beth) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name gabrielle)(zone living_room)(image Alfred) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name elsa)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))

	;(item (type Objects) (name james)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name robert)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name arthur)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name mike)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name richi)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
		


;;;;;;;;;;;ROBOCUP 2016 names
	
	(item (type Objects) (name choco_syrup)(zone bedside)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name bisquits)(zone bedside)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name baby_sweets)(zone bedside)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name egg)(zone bedside)(image bedside)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name pretzels)(zone desk)(image snacks)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pringles)(zone desk)(image snacks)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name beer)(zone bookcase)(image drinks)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coconout_milk)(zone bookcase)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name coke)(zone bookcase)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name tea)(zone bookcase)(image tea)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name apple)(zone sideshelf)(image food)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name paprika)(zone sideshelf)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pumper_nickel)(zone sideshelf)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name shampoo)(zone living_shelf)(image toilet)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name soap)(zone living_shelf)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name sponge)(zone living_shelf)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cloth)(zone living_shelf)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name bowl)(zone sink)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name tray)(zone sink)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name plate)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name snacks)(zone sink)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name candies)(zone sink)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name food)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name drinks)(zone sink)(image milk)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name toiletries)(zone sink)(image coke)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name containers)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	(item (type Objects) (name cloth)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name microwave)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
	
	
	(item (type Objects) (name emma)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name taylor)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name sophia)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name isabella)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name ava)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robin)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name emily)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name angel)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name madison)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charlotte)(zone living_room)(image Anna) (attributes pick)(pose -1.87 8.64 0.0))

	(item (type Objects) (name noah)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name liam)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mason)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jacob)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name william)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name ethan)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name noah)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alexander)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name james)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name daniel)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))


	( item (type Room) (name office) (pose -3.55 -3.0 0.0))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0))


	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))
	(item (type Furniture) (name bedside)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))	
	(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession office)(attributes no_visited))
	(item (type Furniture) (name drawer)(pose -3.55 -3.0 0.0) (possession office)(attributes no_visited))
	(item (type Furniture) (name bar)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name sink)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name cupboard)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name sideshelf)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name tv_stand)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))
	(item (type Furniture) (name living_shelf)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))
	(item (type Furniture) (name living_table)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))
	(item (type Furniture) (name cabinet)(pose -3.55 -3.0 0.0) (possession corridor)(attributes no_visited))	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Rooms definitions
	
	;( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	;( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))

	( item (type Door) (name entrance) (pose -3.55 -3.0 0.0))
	( item (type Door) (name exit)(pose -1.87 8.64 0.0))
	
; Humans definitions
	;( Human (name Mother)(room bedroom)(pose 79.048340 76.107002 0.0))
	;( Human (name Father)(room kitchen)(pose 9.048340 6.107002 0.0))
	;( Human (name You)(room livingroom)(pose 9.048340 6.107002 0.0))

; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))

; Furniture definitions
	( item (type Furniture) (name cubestable) )

	;(item (type Furniture) (name kitchen_table)(pose -3.55 -3.0 0.0))
	;(item (type Furniture) (name stove)(pose -3.55 -3.0 0.0))
	;(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0))

	;(item (type Furniture) (name dinner_table)(pose -3.55 -3.0 0.0))

	;(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0))
	;(item (type Furniture) (name side_table)(pose -3.55 -3.0 0.0))
	
	;(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0))
	;(item (type Furniture) (name couch)(pose -3.55 -3.0 0.0))
	;(item (type Furniture) (name coffe_table)(pose -3.55 -3.0 0.0))
	
	
	
	
	;;;;;
	;(item (type Objects) (name table) (image table)( attributes no-pick brown)(pose -3.55 -3.0 0.0))
	(item (type Person) (name man) (image man)(zone living_room)(pose -1.87 8.64 0.0))
	(item (type Person) (name man_call) (image man)(zone living_room)(pose -1.87 8.64 0.0))

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
	;(item (type Furniture) (name shelf) (status no_ask))
	( item (type Room) (name current_loc) (pose -3.55 -3.0 0.0))

)


