;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;
;;;; virbot_default_facts.clp
;;;;
;;;;	Julio Cruz
;;;;	24/May/2019
;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deffacts init-skts-blackboard
        (address BLACKBOARD "localhost" )
        ;(address BLACKBOARD "148.205.199.80" )
        (port_out BLACKBOARD  2300)

	; Network definitions
	(open-network BLACKBOARD)
)


(deffacts Initial-state-default-facts

        (item (type Category) (name nil) (zone bed))
        (item (type Category) (name nil) (zone entrance))
        (item (type Category) (name nil) (zone desk))
        (item (type Category) (name nil) (zone dining_table))
        (item (type Category) (name nil) (zone exit))
        (item (type Category) (name nil) (zone couch))
        (item (type Category) (name nil) (zone sink))
        (item (type Category) (name nil) (zone dishwasher))

        ;;;;; Property
        (item (type Property) (name biggest) (zone bookcase))
        (item (type Property) (name smallest) (zone bookcase))
        (item (type Property) (name heaviest) (zone bookcase))
        (item (type Property) (name lightest) (zone bookcase))
        (item (type Property) (name largest) (zone bookcase))
        (item (type Property) (name thinnest) (zone bookcase))
        (item (type Property) (name tallest) (zone bookcase))

	;;;;; Abspos
        (item (type Abspos) (name left_most))
        (item (type Abspos) (name right_most))
        
	;;;;; Relpos
	(item (type Relpos) (name at_the_right_of))
	(item (type Relpos) (name at_the_left_of))
	(item (type Relpos) (name on_top_of))
	(item (type Relpos) (name above))
	(item (type Relpos) (name behind))
	(item (type Relpos) (name under))

	
	;;; gestures
        (item (type Gesture) (name waving) (image _))
        (item (type Gesture) (name raising_their_left_arm) (image _))
        (item (type Gesture) (name raising_their_right_arm) (image _))
        (item (type Gesture) (name pointing_to_the_left) (image _))
        (item (type Gesture) (name pointing_to_the_right) (image _))

	;;; Global
        (item (type Gender) (name gender))
        (item (type Pose) (name pose))
        (item (type Gesture) (name gesture))
	
	;;; Genders
        (item (type Gender) (name man))
        (item (type Gender) (name woman))
        (item (type Gender) (name boy))
        (item (type Gender) (name girl))
	(item (type Gender) (name child))
        (item (type Gender) (name male_person))
        (item (type Gender) (name female_person))

        (item (type Gender) (name men))
        (item (type Gender) (name women))
        (item (type Gender) (name boys))
        (item (type Gender) (name girls))
        (item (type Gender) (name male))
        (item (type Gender) (name female))
						
        (item (type Gender) (name children))
        (item (type Gender) (name adults))
        (item (type Gender) (name elders))
        (item (type Gender) (name males))
        (item (type Gender) (name females))

	;;Poses
        (item (type Pose) (name sitting)(image _))
        (item (type Pose) (name standing)(image _))
        (item (type Pose) (name lying)(image _))

	;;outfit
        (item (type Outfit) (name shoes)(image wearing))
        (item (type Outfit) (name pants)(image wearing))
        (item (type Outfit) (name t-shirt)(image wearing))
        (item (type Outfit) (name shirt)(image wearing))
        (item (type Outfit) (name blouse)(image wearing))
        (item (type Outfit) (name sweater)(image wearing))
        (item (type Outfit) (name coat)(image wearing))
        (item (type Outfit) (name jacket)(image wearing))
        (item (type Outfit) (name t-shirts)(image wearing))
        (item (type Outfit) (name shirts)(image wearing))
        (item (type Outfit) (name blouses)(image wearing))
        (item (type Outfit) (name sweaters)(image wearing))
        (item (type Outfit) (name coats)(image wearing))
        (item (type Outfit) (name jackets)(image wearing))
        (item (type Outfit) (name hat)(image wearing))
        (item (type Outfit) (name glasses)(image wearing))
        (item (type Outfit) (name necklace)(image wearing))
        (item (type Outfit) (name tie)(image wearing))
        (item (type Outfit) (name earrings)(image wearing))

	;;colors
        (item (type Color) (name blue)(image dressed_in))
        (item (type Color) (name yellow)(image dressed_in))
        (item (type Color) (name black)(image dressed_in))
        (item (type Color) (name white)(image dressed_in))
        (item (type Color) (name red)(image dressed_in))
        (item (type Color) (name gray)(image dressed_in))
        (item (type Color) (name orange)(image dressed_in))

        ;;;;;DOORS
        (item (type Door) (quantity 1) (room bedroom))
        (item (type Door) (quantity 0) (room living_room))
        (item (type Door) (quantity 1) (room office))
        (item (type Door) (quantity 1) (room kitchen))
        ;(item (type Door) (quantity 2) (room corridor))
        ;(item (type Door) (quantity 0) (room dining_room))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Rooms definitions
	
	;( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	;( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))

	( item (type Door) (name entrance)(possession Dummy)(attributes no_visited)(room Dummy) (pose -3.55 -3.0 0.0))
	( item (type Door) (name exit)(possession Dummy)(attributes no_visited)(room Dummy)(pose -1.87 8.64 0.0))


	
; Humans definitions
	;( Human (name Mother)(room bedroom)(pose 79.048340 76.107002 0.0))
	;( Human (name Father)(room kitchen)(pose 9.048340 6.107002 0.0))
	;( Human (name You)(room livingroom)(pose 9.048340 6.107002 0.0))

; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))
	( Arm (name right) (bandera false))
	( Arm (name left) (bandera true))
	( item (name right))
	( item (name left))

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
	( item (type Door) (name front) (image entrance)(status closed) (possession cupboard))
	( item (type Door) (name main) (image entrance)(status closed) (possession cupboard))
	( item (type Door) (name back) (image exit)(status closed) (possession cupboard))
	( item (type Door) (name rear) (image exit)(status closed) (possession cupboard))
	( item (type Zone) (name tablezone) (status closed) (possession table))

; Human-Robot-Interface
	;( item (type H-R-Interface) (name naturallanguage) (status nil) )
;;;;;;;;;new forniture for task "ask for"
	;(item (type Furniture) (name cupboard)(pose -1.3 6.05 0.0));;;commented for find category task
	;(item (type Furniture) (name fridge)(pose -4.07 6.64 0.0));;;commented for find category taks
	(item (type Furniture) (name table)(pose -1.55 4.03 0.0))
	
	(item (type question) (name question_1) (status no_ask) (possession table))
	(item (type question) (name question_2) (status no_ask) (possession table))
	(item (type question) (name question_3) (status no_ask) (possession table))
	(item (type question) (name question_4) (status no_ask) (possession table))
	(item (type question) (name question_5) (status no_ask) (possession table))
	(item (type question) (name question_6) (status no_ask) (possession table))
	(item (type question) (name question_7) (status no_ask) (possession table))
	(item (type question) (name question_8) (status no_ask) (possession table))
	(item (type question) (name question_9) (status no_ask) (possession table))
	
	(item (type Door) (name exitdoor) (status no_ask) (possession table))
	;(item (type Furniture) (name shelf) (status no_ask))
	( item (type Room) (name current_loc) (pose -3.55 -3.0 0.0))
	( item (type Furniture) (name bar) (possession living_room) (room living_room)(pose -3.55 -3.0 0.0))
	(item (type Door) (name arena) (status no_ask) (possession table))
        (item (type Speech) (name speech_1) (image i_am_ready_for_recieve_a_new_command))
        (item (type Speech) (name speech_2) (image i_finish_the_test))
	(item (type Speech) (name speech))
        (item (type Speech) (name join_dark_side) (image i_always_belonged_to_the_dark_side))
	
	( item (type Objects) (name object)(zone nil)(pose 1.048340 1.107002 0.0))

;;;;;; hecho para guardar el satus de la confirmacion
	(item (type Confirmation) (name conf) (status nil))
	(item (type Confirmation) (name incomplete) (status nil))
;;;;;;; objeto para guardar status de terminacion de un plan
	(item (type Objects) (name finish_objetive)(status nil))
)

