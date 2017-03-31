
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


	

;;;;;;;;;;;TMR 2017 names

	;;; DRINKS
	(item (type Objects) (name juice)(zone bedside)(image juice)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name boing)(zone bedside)(image boing)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name jumex)(zone bedside)(image jumex)(attributes pick)(pose -3.55 -3.0 0.0))

	;;; sweets
	(item (type Objects) (name cereal)(zone bedside)(image cereal)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cookies)(zone bedside)(image cookies)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name kinder)(zone bedside)(image kinder)(attributes pick)(pose -3.55 -3.0 0.0))

	;;; SNACKS
	(item (type Objects) (name corn)(zone bedside)(image corn)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name cheese)(zone bedside)(image cheese)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name pringles)(zone bedside)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0))
	(item (type Objects) (name saltines)(zone bedside)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0))
	

	(item (type Objects) (name milk)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))

	;;;; Fameles
	(item (type Objects) (name emma)(zone living_room)(image emma) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charlotte)(zone living_room)(image charlotte) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name veronica)(zone living_room)(image veronica) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name leslie)(zone living_room)(image leslie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mary)(zone living_room)(image mary) (attributes pick)(pose -1.87 8.64 0.0))

	;;; Males
	(item (type Objects) (name charly)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name peter)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name frank)(zone living_room)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name john)(zone kitchen)(image Charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alexander)(zone living_room)(image alexander) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mark)(zone living_room)(image mark) (attributes pick)(pose -1.87 8.64 0.0))


	;;; locations
	( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))
	( item (type Room) (name hall)(pose -1.87 8.64 0.0))


	;;; furniture
	(item (type Furniture) (name bedroom_table)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited))	
	(item (type Furniture) (name side_table)(pose -3.55 -3.0 0.0) (possession dining_room)(attributes no_visited))
	(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0) (possession hall)(attributes no_visited))
	(item (type Furniture) (name dinner_table)(pose -3.55 -3.0 0.0) (possession dining_room)(attributes no_visited))
	(item (type Furniture) (name cabinet)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))	
	(item (type Furniture) (name shelf)(pose -3.55 -3.0 0.0) (possession dining_room)(attributes no_visited))
	(item (type Furniture) (name stove)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0) (possession kitchen)(attributes no_visited))
	(item (type Furniture) (name tv)(pose -3.55 -3.0 0.0) (possession living_room)(attributes no_visited))	

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


