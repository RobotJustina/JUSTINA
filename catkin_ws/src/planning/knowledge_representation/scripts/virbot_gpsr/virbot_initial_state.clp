
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
	;(item (type Objects) (name chips)(zone desk)(image chips)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 1) (size 4) (wide 4) (height 3)(color yellow) (quantity 1))
	;(item (type Objects) (name senbei)(zone desk)(image senbei)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 2) (size 3) (wide 1) (height 1)(color brown) (quantity 1))
	;(item (type Objects) (name pringles)(zone desk)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 3) (size 2) (wide 3)(height 2)(color red) (quantity 1))
	;(item (type Objects) (name peanuts)(zone desk)(image peanuts)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 1) (wide 2) (height 4)(color blue) (quantity 1))

	(item (type Objects) (name candy)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen)(weight 8)(size 12)(wide 23)(height 11)(color red))
	(item (type Objects) (name chewing_gum)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen)(weight 14)(size 11)(wide 14)(height 13)(color white))
	(item (type Objects) (name cup_star)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen)(weight 7)(size 15)(wide 19)(height 15)(color white))
	(item (type Objects) (name curry)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen)(weight 6)(size 14)(wide 17)(height 14)(color white))
	(item (type Objects) (name fries)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen)(weight 5)(size 18)(wide 8)(height 18)(color red))
	(item (type Objects) (name jelly)(zone kitchen_rack)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snack)(room kitchen) (weight 16)(size 9)(wide 7)(height 9)(color orange))
	

	;(item (type Objects) (name chocolate_egg)(zone center_table)(image chocolate_egg)(attributes pick)(pose -3.55 -3.0 0.0)(category candies)(smallest yes))

	;;; DRINKS
	(item (type Objects) (name aquarius)(zone kitchen_counter)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0) (category drink)(weight 25)(size 16)(wide 15)(height 16)(color blue))
	(item (type Objects) (name coke)(zone kitchen_counter)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0) (category drink)(weight 19)(size 20)(wide 10)(height 20)(color black))
	(item (type Objects) (name cold_brew)(zone kitchen_counter)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0) (category drink)(weight 18)(size 17)(wide 11)(height 17)(color blue))
	(item (type Objects) (name green_tea)(zone kitchen_counter)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0) (category drink)(weight 26)(size 24)(wide 9)(height 24)(color green)(heaviest yes))

	;;; FOOD

	(item (type Objects) (name bread)(zone kitchen_shelf)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(weight 23)(size 26)(wide 24)(height 26)(color red)(biggest yes))
	(item (type Objects) (name corn)(zone kitchen_shelf)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(weight 21)(size 8)(wide 22)(height 5)(color yellow))
	(item (type Objects) (name onion)(zone kitchen_shelf)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(weight 9)(size 5)(wide 16)(height 7)(color white))
	(item (type Objects) (name radish)(zone kitchen_shelf)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(weight 24)(size 23)(wide 21)(height 21)(color white))


	;;;; CONTAINERS
	(item (type Objects) (name bowl)(zone left_rack)(image bowl)(attributes pick)(pose -3.55 -3.0 0.0) (category container)(weight 12)(size 19)(wide 25)(height 10)(color gray)(biggest yes))
	(item (type Objects) (name plate)(zone left_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category container)(weight 11)(size 4)(wide 26)(height 4)(color white))
	(item (type Objects) (name soup_container)(zone left_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category container)(weight 4)(size 10)(wide 16)(height 12)(color black))

	;;;; FRUIT
	(item (type Objects) (name apple)(zone bistro_table)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category fruit)(weight 10)(size 7)(wide 12)(height 8)(color red))	
	(item (type Objects) (name orange_fruit)(zone bistro_table)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category fruit)(weight 15)(size 6)(wide 20)(height 6)(color orange))

	;;;; CLEANNING STUFF
	(item (type Objects) (name asience)(zone right_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cleaning_stuff)(weight 20)(size 21)(wide 6)(height 22)(color red))
	(item (type Objects) (name hair_spray)(zone right_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cleaning_stuff)(weight 17)(size 22)(wide 5)(height 25)(color green))
	(item (type Objects) (name moisturizer)(zone right_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cleaning_stuff)(weight 13)(size 13)(wide 4)(height 19)(color white))
	(item (type Objects) (name shampoo)(zone right_rack)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cleaning_stuff)(weight 22)(size 25)(wide 18)(height 23)(color green))
	
	;;;; CUTLERY
	(item (type Objects) (name chopstick)(zone sideboard)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cutlery)(weight 3)(size 2)(wide 3)(height 1)(color white))
	(item (type Objects) (name fork)(zone sideboard)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cutlery)(weight 2)(size 3)(wide 2)(height 3)(color white))
	(item (type Objects) (name spoon)(zone sideboard)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category cutlery)(weight 1)(size 1)(wide 1)(height 2)(color white)(smallest yes)(lightest yes))


	(item (type Objects) (name milk)(zone left_rack)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0))
        (item (type Objects) (name juice)(zone left_rack)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0)(category drink))

        ;;;;;; Category 
        (item (type Category) (name snack) (zone kitchen_rack)(quantity 6)(biggest fries)(smallest jelly) (heaviest jelly) (lightest fries))
        (item (type Category) (name fruit) (zone bistro_table)(quantity 2) (biggest apple) (smallest orange_fruit) (heaviest orange_fruit) (lightest apple))
        (item (type Category) (name drink) (zone kitchen_counter)(quantity 4)(biggest green_tea) (smallest aquarius) (heaviest green_tea) (lightest cold_brew))
        (item (type Category) (name food) (zone kitchen_shelf)(quantity 4)(biggest bread) (smallest corn) (heaviest radish) (lightest onion))
        (item (type Category) (name cutlery) (zone sideboard)(quantity 3)(biggest fork) (smallest spoon) (heaviest chopstick) (lightest spoon))
        (item (type Category) (name container) (zone left_rack)(quantity 3)(biggest bowl) (smallest plate) (heaviest bowl) (lightest soup_container))
        (item (type Category) (name cleaning_stuff) (zone right_rack)(quantity 4)(biggest shampoo) (smallest moisturizer) (heaviest shampoo) (lightest moisturizer))

        (item (type Category) (name nil) (zone desk))
        (item (type Category) (name nil) (zone little_desk))
        (item (type Category) (name nil) (zone teepee))
        (item (type Category) (name nil) (zone bed))
        (item (type Category) (name nil) (zone entrance_shelf))
        (item (type Category) (name nil) (zone bookcase))
        (item (type Category) (name nil) (zone sofa))
        (item (type Category) (name nil) (zone coffee_table))
        (item (type Category) (name nil) (zone tv))
        (item (type Category) (name nil) (zone kitchen_table))
        (item (type Category) (name nil) (zone left_planks))
        (item (type Category) (name nil) (zone right_planks))
        (item (type Category) (name nil) (zone balcony_shelf))
        (item (type Category) (name nil) (zone fridge))

        ;;;;; Property
        (item (type Property) (name biggest) (zone bed))
        (item (type Property) (name smallest) (zone bed))
        (item (type Property) (name heaviest) (zone bed))
        (item (type Property) (name lightest) (zone bed))
        (item (type Property) (name largest) (zone bed))
        (item (type Property) (name thinnest) (zone bed))

;;;;;;;;;;PERSONS

	;;;; Fameles
	(item (type Objects) (name emma)(zone living_room)(image emma) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name olivia)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name sophia)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name ava)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name isabella)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mia)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name abigail)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name emily)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charlotte)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name harper)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	
	;;; Males
	(item (type Objects) (name noah)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name liam)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mason)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jacob)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name william)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name ethan)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name james)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name alexander)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name benjamin)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	;;;gestures
        (item (type Gesture) (name waving))
        (item (type Gesture) (name rising_left_arm))
        (item (type Gesture) (name rising_right_arm))
        (item (type Gesture) (name pointing_left))
        (item (type Gesture) (name pointing_right))

	;;;Global
        (item (type Gender) (name gender))
        (item (type Pose) (name pose))
        (item (type Gesture) (name gesture))
	

	;;;Genders
        (item (type Gender) (name man))
        (item (type Gender) (name woman))
        (item (type Gender) (name boy))
        (item (type Gender) (name girl))
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
        (item (type Pose) (name sitting))
        (item (type Pose) (name standing))
        (item (type Pose) (name lying))

;;;;;;;; LOCATIONS

	;;;ROOM
	;( item (type Room) (name bedroom) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))
	;( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	;( item (type Room) (name office) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 1))
	;( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 8) (quantitys 2))
	;( item (type Room) (name corridor)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 1))
	;( item (type Room) (name bathroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

	( item (type Room) (name entrance)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 2) (quantitys 1))
	( item (type Room) (name bedroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 2))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0)(quantity 18) (quantitys 4))
	( item (type Room) (name balcony)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

	;;;placment
	;(item (type Furniture) (name bedside)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited) (room bedroom))
	;(item (type Furniture) (name living_shelf)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited) (room living_room))
        ;(item (type Furniture) (name living_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited) (room living_room))
	;(item (type Furniture) (name bar)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	;(item (type Furniture) (name drawer)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	;(item (type Furniture) (name cupboard)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name cupboard_1)(pose -3.55 -3.0 0.0)(possession bathroom)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name sideshelf)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))

	;;;;beacons
	;(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited)(room bedroom))
	;(item (type Furniture) (name tv_stand)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	;(item (type Furniture) (name center_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	;(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession office)(attributes no_visited)(room office))
	;(item (type Furniture) (name sink)(pose -3.55 -3.0 0.0)(possession office)(attributes no_visited)(room office))
	;(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	;(item (type Furniture) (name cabinet)(pose -3.55 -3.0 0.0)(possession corridor)(attributes no_visited)(room corridor))


	(item (type Furniture) (name desk)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name left_rack)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name right_rack)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name sideboard)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name kitchen_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name little_desk)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited)(room bedroom))
	(item (type Furniture) (name teepee)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited)(room bedroom))
	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bedroom)(attributes no_visited)(room bedroom))
	(item (type Furniture) (name entrance_shelf)(pose -3.55 -3.0 0.0)(possession entrance)(attributes no_visited)(room entrance))
	(item (type Furniture) (name kitchen_shelf)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name bookcase)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name coffee_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name tv)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name bistro_table)(pose -3.55 -3.0 0.0)(possession balcony)(attributes no_visited)(room balcony))
	(item (type Furniture) (name left_planks)(pose -3.55 -3.0 0.0)(possession balcony)(attributes no_visited)(room balcony))
	(item (type Furniture) (name right_planks)(pose -3.55 -3.0 0.0)(possession balcony)(attributes no_visited)(room balcony))
	(item (type Furniture) (name balcony_shelf)(pose -3.55 -3.0 0.0)(possession balcony)(attributes no_visited)(room balcony))
	(item (type Furniture) (name kitchen_counter)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name kitchen_rack)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
        ;;;;;DOORS
        (item (type Door) (quantity 1) (room living_room))
        (item (type Door) (quantity 0) (room corridor))
        (item (type Door) (quantity 1) (room kitchen))
        (item (type Door) (quantity 1) (room bedroom))
        (item (type Door) (quantity 1) (room entrance))
        (item (type Door) (quantity 0) (room balcony))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; Rooms definitions
	
	;( item (type Room) (name dining_room) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name living_room)(pose -1.87 8.64 0.0))
	;( item (type Room) (name kitchen) (pose -3.55 -3.0 0.0))
	;( item (type Room) (name bedroom)(pose -1.87 8.64 0.0))

	;( item (type Door) (name entrance) (pose -3.55 -3.0 0.0))
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
	
	;;;;;
	;(item (type Objects) (name table) (image table)( attributes no-pick brown)(pose -3.55 -3.0 0.0))
	(item (type Person) (name man) (image man)(zone living_room)(pose -1.87 8.64 0.0))
        (item (type Person) (name man_guide) (image man)(zone living_room)(pose -1.87 8.64 0.0))
	(item (type Person) (name man_call) (image man)(zone living_room)(pose -1.87 8.64 0.0))
        (item (type Person) (name person) (image man)(zone living_room)(pose -1.87 8.64 0.0))

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
        (item (type Speech) (name join_dark_side) (image i_always_belonged_to_the_dark_side))
)

