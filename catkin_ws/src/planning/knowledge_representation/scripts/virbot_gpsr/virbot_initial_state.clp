
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


	

;;;;;;;;;;;NAGOYA 2017 names

	;;; DRINKS
	(item (type Objects) (name chips)(zone bedside)(image chips)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name senbei)(zone bedside)(image senbei)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pringles)(zone bedside)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name peanuts)(zone bedside)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0))

	;;; sweets
	(item (type Objects) (name chocolate)(zone center_table)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name manju)(zone center_table)(image manju)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name mints)(zone center_table)(image mints)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name chocolate_egg)(zone center_table)(image chocolate_egg)(attributes pick)(pose -3.55 -3.0 0.0))

	;;; SNACKS
	(item (type Objects) (name corn)(zone desk)(image corn)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cheese)(zone desk)(image cheese)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pringles)(zone desk)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name saltines)(zone desk)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0))

	;;; FOOD
	(item (type Objects) (name noodles)(zone fridge)(image noodles)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name apple)(zone fridge)(image apple)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name paprika)(zone fridge)(image paprika)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name watermelon)(zone fridge)(image watermelon)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name sushi)(zone fridge)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0))

	;;;; TOILETRIES
	(item (type Objects) (name shampoo)(zone cupboard)(image shampoo)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name soap)(zone cupboard)(image soap)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cloth)(zone cupboard)(image cloth)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name sponge)(zone cupboard)(image sponge)(attributes pick)(pose -3.55 -3.0 0.0))

	;;;; CONTAINERS
	(item (type Objects) (name bowl)(zone counter)(image bowl)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name tray)(zone counter)(image tray)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name plate)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0))
	

	(item (type Objects) (name milk)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

;;;;;;;;;;PERSONS

	;;;; Fameles
	(item (type Objects) (name hanna)(zone living_room)(image hanna) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name barbara)(zone living_room)(image barbara) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name samantha)(zone living_room)(image samantha) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name erika)(zone living_room)(image erika) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name sophie)(zone living_room)(image sophie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jackie)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))

	;;; Males
	(item (type Objects) (name ken)(zone living_room)(image ken) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name erik)(zone living_room)(image erik) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name samuel)(zone living_room)(image samuel) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name skyler)(zone kitchen)(image skyler) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name brian)(zone living_room)(image brian) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name thomas)(zone living_room)(image thomas) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name edward)(zone living_room)(image edward) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image michael) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charlie)(zone living_room)(image charlie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alex)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))


;;;;;;;; LOCATIONS

	;;;ROOMS
	( item (type Room) (name bedroom) (pose -3.55 -3.0 0.0))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	( item (type Room) (name office) (pose -3.55 -3.0 0.0))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0))
	( item (type Room) (name bathrooom)(pose -1.87 8.64 0.0))

	;;;placment
	(item (type Furniture) (name bedside)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))
	(item (type Furniture) (name living_shelf)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited))
	(item (type Furniture) (name bar)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited))
	(item (type Furniture) (name drawer)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited))
	(item (type Furniture) (name cupboard)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name sideshelf)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))

	;;;;beacons
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))
	(item (type Furniture) (name tv_stand)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited))
	(item (type Furniture) (name center_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited))
	(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession office)(attributes no_visited))
	(item (type Furniture) (name sink)(pose -3.55 -3.0 0.0)(possession office)(attributes no_visited))
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited))
	(item (type Furniture) (name cabinet)(pose -3.55 -3.0 0.0)(possession corridor)(attributes no_visited))

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
	( Arm (name right)(bandera false))
	( Arm (name left) (bandera true))

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
	
	(item (type question) (name question_1) (status no_ask) (possession table))
	(item (type question) (name question_2) (status no_ask) (possession table))
	(item (type question) (name question_3) (status no_ask) (possession table))
	(item (type Door) (name exitdoor) (status no_ask) (possession table))
	;(item (type Furniture) (name shelf) (status no_ask))
	( item (type Room) (name current_loc) (pose -3.55 -3.0 0.0))
	(item (type Door) (name arena) (status no_ask) (possession table))
)


