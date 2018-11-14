
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

	;;; SNACKS
	;(item (type Objects) (name chips)(zone desk)(image chips)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 1) (size 4) (wide 4) (height 3)(color yellow) (quantity 1) (grasp 1))
	;(item (type Objects) (name senbei)(zone desk)(image senbei)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 2) (size 3) (wide 1) (height 1)(color brown) (quantity 1) (grasp 2))
	;(item (type Objects) (name pringles)(zone desk)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 3) (size 2) (wide 3)(height 2)(color red) (quantity 1) (grasp 31))
	;(item (type Objects) (name peanuts)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 1) (wide 2) (height 4)(color blue) (quantity 1) (grasp 6))
	(item (type Objects) (name crackers) (zone bookcase) (image crackers) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room living_room)(weight 8)(grasp 17)(size 22)(height 22)(wide 15)(color orange))
	(item (type Objects) (name potato_chips) (zone bookcase) (image potato_chips) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room living_room)(weight 19)(size 24)(height 25) (wide 21)(color black)(grasp 11))
	(item (type Objects) (name pringles) (zone bookcase) (image pringles) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room living_room)(grasp 12)(weight 18)(size 21)(height 24)(wide 10)(color green))

	;;; DRINKS
	(item (type Objects) (name chocolate_drink)(zone counter)(image chocolate_drink)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (room kitchen) (color brown)(grasp 15)(size 17) (wide 4) (height 23) (weight 25)(heaviest yes))
	(item (type Objects) (name grape_juice)(zone counter)(image grape_juice)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (room kitchen) (color purple)(grasp 25)(size 8) (wide 2) (height 19) (weight 22))
	(item (type Objects) (name coke)(zone counter)(image coke)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (room kitchen) (color red)(size 13) (wide 3) (height 16) (weight 23)(grasp 13))
	(item (type Objects) (name orange_juice)(zone counter)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (room kitchen) (color orange)(size 7) (wide 1) (height 18) (weight 21)(grasp 26))
	(item (type Objects) (name sprite)(zone counter)(image sprite)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (room kitchen) (color green)(size 14) (wide 5) (height 17) (weight 24)(grasp 14))


	;;; FOOD
	;(item (type Objects) (name noodles)(zone fridge)(image noodles)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	(item (type Objects) (name cereal) (zone cupboard) (image cereal) (attributes pick) (pose 0.0 0.0 0.0) (category food) (room kitchen)(grasp 24)(weight 7)(size 10)(height 20)(wide 9)(color blue))
	(item (type Objects) (name noodles) (zone cupboard) (image noodles) (attributes pick) (pose 0.0 0.0 0.0) (category food) (room kitchen)(grasp 20)(weight 12)(size 12)(height 6)(wide 17)(color white))
	(item (type Objects) (name sausages) (zone cupboard) (image sausages) (attributes pick) (pose 0.0 0.0 0.0) (category food) (room kitchen)(grasp 23)(weight 15)(size 9)(height 12)(wide 8)(color blue))


	;;;; CLEANING STUFF
	(item (type Objects) (name scrubby) (zone side_table) (image scrubby) (attributes pick) (pose 0.0 0.0 0.0) (category cleaning_stuff)(room bedroom) (grasp 19)(weight 6)(size 15)(height 21)(wide 11)(color yellow))
	(item (type Objects) (name cloth) (zone side_table) (image cloth) (attributes pick) (pose 0.0 0.0 0.0) (category cleaning_stuff)(room bedroom)(weight 2)(grasp 1)(size 6)(height 2)(wide 18)(color purple))
	(item (type Objects) (name sponge) (zone side_table) (image sponge) (attributes pick) (pose 0.0 0.0 0.0) (category cleaning_stuff)(room bedroom)(grasp 18)(weight 5)(size 11)(height 8)(wide 14)(color blue))
	(item (type Objects) (name cascade_pod) (zone side_table) (image cascade_pod) (attributes pick) (pose 0.0 0.0 0.0) (category cleaning_stuff)(room bedroom)(grasp 0)(weight 0)(size 0)(height 0)(wide 0)(color white))


	;;;; CONTAINERS
	(item (type Objects) (name tray)(zone end_table)(image tray)(attributes pick)(pose -3.55 -3.0 0.0) (category containers)(room living_room) (grasp 4)(weight 17)(size 23)(height 7)(wide 25)(color white))
	(item (type Objects) (name basket)(zone end_table)(image basket)(attributes pick)(pose -3.55 -3.0 0.0) (category containers)(room living_room)(grasp 2)(weight 20)(size 25)(height 14)(wide 24)(color beige))
	(item (type Objects) (name bag)(zone end_table)(image bag)(attributes pick)(pose -3.55 -3.0 0.0)(weight 0)(height 26)(size 26) (wide 0) (color pink)(grasp 3)(category containers)(room living_room)(biggest yes))

	;;;;; FRUITS
	(item (type Objects) (name apple) (zone bookcase) (image apple) (attributes pick) (pose 0.0 0.0 0.0) (category fruits) (room living_room)(grasp 22)(weight 14)(size 5)(height 10)(wide 7)(color red))
	(item (type Objects) (name orange) (zone bookcase) (image orange) (attributes pick) (pose 0.0 0.0 0.0) (category fruits) (room living_room)(grasp 21)(weight 13)(size 4)(height 9)(wide 6)(color orange))
	(item (type Objects) (name paprika) (zone bookcase) (image paprika) (attributes pick) (pose 0.0 0.0 0.0) (category fruits) (room living_room)(grasp 16)(weight 16)(size 18)(height 15)(wide 12)(color red))

	;;;; CUTTLERY
	(item (type Objects) (name spoon) (zone storage_table) (image spoon) (attributes pick) (pose 0.0 0.0 0.0) (category cutlery) (room kitchen)(grasp 10)(weight 4)(size 3)(height 4)(wide 20)(color green))
	(item (type Objects) (name fork) (zone storage_table) (image fork) (attributes pick) (pose 0.0 0.0 0.0) (category cutlery) (room kitchen)(grasp 9)(weight 3) (size 2)(height 3)(wide 19)(color green))
	(item (type Objects) (name knife) (zone storage_table) (image knife) (attributes pick) (pose 0.0 0.0 0.0) (category cutlery) (room kitchen)(grasp 8)(weight 1) (size 1)(height 1)(wide 22)(color green)(smallest yes)(lightest yes))

	;;;; TABLEWARE
	(item (type Objects) (name dish) (zone storage_table) (image dish) (attributes pick) (pose 0.0 0.0 0.0) (category tableware) (room dining_room)(grasp 7)(weight 10)(size 20)(height 5)(wide 23)(color green))
	(item (type Objects) (name bowl) (zone storage_table) (image bowl) (attributes pick) (pose 0.0 0.0 0.0) (category tableware) (room dining_room)(grasp 5)(weight 9)(size 19)(height 11)(wide 16)(color green))
	(item (type Objects) (name cup) (zone storage_table) (image cup) (attributes pick) (pose 0.0 0.0 0.0) (category tableware) (room dining_room)(grasp 6)(weight 11)(size 16)(height 13)(wide 13)(color green))

	;(item (type Objects) (name milk)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0)(category drinks)(grasp 33))
        ;(item (type Objects) (name juice)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0)(category drinks) (grasp 32))
	;(item (type Objects) (name soup) (zone sink) (image orange_juice)(attributes pick)(pose 0.0 0.0 0.0) (category food) (grasp 34))

        ;;;;;; Category 
        (item (type Category) (name snacks) (zone bookcase)(quantity 3)(biggest potato_chips)(smallest pringles) (heaviest potate_chips) (lightest crackers))
        ;(item (type Category) (name candies) (zone center_table)(quantity 4) (biggest chocolate_bar) (smallest mints) (heaviest chocolate_bar) (lightest mints))
        (item (type Category) (name drinks) (zone counter)(quantity 5)(biggest sprite) (smallest orange_juice) (heaviest sprite) (lightest orange_juice))
        (item (type Category) (name food) (zone cupboard)(quantity 3)(biggest noodles) (smallest sausages) (heaviest sausages) (lightest cereal))
        (item (type Category) (name cleaning_stuff) (zone side_table)(quantity 3)(biggest scrubby) (smallest cloth) (heaviest scrubby) (lightest cloth))
        (item (type Category) (name containers) (zone end_table)(quantity 2)(biggest basket) (smallest tray) (heaviest basket) (lightest tray))
        (item (type Category) (name cutlery) (zone storage_table)(quantity 3)(biggest spoon) (smallest knife) (heaviest spoon) (lightest knife))
	(item (type Category) (name tableware) (zone storage_table) (quantity 3) (biggest dish) (smallest cup) (heaviest cup) (lightest bowl))
	(item (type Category) (name fruits) (zone bookcase) (quantity 3) (biggest paprika) (smallest orange) (heaviest paprika) (lightest orange))

        (item (type Category) (name nil) (zone sink))
        (item (type Category) (name nil) (zone dishwasher))
        (item (type Category) (name nil) (zone dining_table))
        (item (type Category) (name nil) (zone bed))
        (item (type Category) (name nil) (zone desk))
        (item (type Category) (name nil) (zone couch))

        ;;;;; Property
        (item (type Property) (name biggest) (zone end_table))
        (item (type Property) (name smallest) (zone storage_table))
        (item (type Property) (name heaviest) (zone counter))
        (item (type Property) (name lightest) (zone storage_table))
        (item (type Property) (name largest) (zone bookcase))
        (item (type Property) (name thinnest) (zone counter))

;;;;;;;;;;PERSONS

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


;;;;;;;; LOCATIONS

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

	;;;;beacons

        ;;;;;DOORS
        (item (type Door) (quantity 2) (room bedroom))
        (item (type Door) (quantity 2) (room living_room))
        ;(item (type Door) (quantity 3) (room office))
        (item (type Door) (quantity 1) (room kitchen))
        (item (type Door) (quantity 1) (room corridor))
        (item (type Door) (quantity 0) (room dining_room))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Rooms definitions
	
	;( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	;( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))

	( item (type Door) (name entrance)(possession corridor)(attributes no_visited)(room corridor) (pose -3.55 -3.0 0.0))
	( item (type Door) (name exit)(possession living_room)(attributes no_visited)(room living_room)(pose -1.87 8.64 0.0))


	
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

; Human-Robot-Interface
	;( item (type H-R-Interface) (name naturallanguage) (status nil) )
;;;;;;;;;new forniture for task "ask for"
	;(item (type Furniture) (name cupboard)(pose -1.3 6.05 0.0));;;commented for find category task
	;(item (type Furniture) (name fridge)(pose -4.07 6.64 0.0));;;commented for find category taks
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
;;;;;;; objeto para guardar status de terminacion de un plan
	(item (type Objects) (name finish_objetive)(status nil))
)

