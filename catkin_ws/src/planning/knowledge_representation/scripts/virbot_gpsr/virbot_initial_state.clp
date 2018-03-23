
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
	(item (type Objects) (name chocolate)(zone living_table)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 1) (size 4) (wide 9) (height 5)(color red) (quantity 1) (lightest yes) (grasp 6))
	(item (type Objects) (name pringles)(zone living_table)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 4) (size 12) (wide 10) (height 11)(color purple) (quantity 1) (biggest yes)(grasp 6))
	(item (type Objects) (name cookies)(zone living_table)(image cookies)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 3) (size 10) (wide 2) (height 1)(color red) (quantity 1) (grasp 6))
	(item (type Objects) (name crackets)(zone living_table)(image crackets)(attributes pick)(pose -3.55 -3.0 0.0)(category snacks)(room office) (weight 2) (size 11) (wide 12) (height 6)(color brown) (quantity 1) (grasp 6) )

	;;; CANDIES
	;(item (type Objects) (name chocolate_bar)(zone center_table)(image chocolate)(attributes pick)(pose -3.55 -3.0 0.0)(category candies))
	;(item (type Objects) (name manju)(zone center_table)(image manju)(attributes pick)(pose -3.55 -3.0 0.0)(category candies))
	;(item (type Objects) (name mints)(zone center_table)(image mints)(attributes pick)(pose -3.55 -3.0 0.0)(category candies)(lightest yes))
	;(item (type Objects) (name chocolate_egg)(zone center_table)(image chocolate_egg)(attributes pick)(pose -3.55 -3.0 0.0)(category candies)(smallest yes))

	;;; DRINKS
	;(item (type Objects) (name tea)(zone bar)(image corn)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks))
	;(item (type Objects) (name beer)(zone bar)(image cheese)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks))
	;(item (type Objects) (name sake)(zone bar)(image pringles)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks))
	;(item (type Objects) (name coke)(zone bar)(image saltines)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (grasp 30))
	(item (type Objects) (name coke)(zone kitchen_table)(image coke)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (color blue)(grasp 30)(size 6) (wide 3) (height 8) (weight 12)(heaviest yes))
	(item (type Objects) (name coffee)(zone kitchen_table)(image coffee)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (color brown)(grasp 30)(size 7) (wide 11) (height 9) (weight 11))
	(item (type Objects) (name juice)(zone kitchen_table)(image juice)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (color pink)(size 5) (wide 4) (height 7) (weight 8)(grasp 30))
	(item (type Objects) (name milk)(zone kitchen_table)(image milk)(attributes pick)(pose -3.55 -3.0 0.0) (category drinks) (color brown)(size 8) (wide 5) (height 10) (weight 5)(grasp 30))


	;;; FOOD
	;(item (type Objects) (name noodles)(zone fridge)(image noodles)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	;(item (type Objects) (name apple)(zone fridge)(image apple)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	;(item (type Objects) (name paprika)(zone fridge)(image paprika)(attributes pick)(pose -3.55 -3.0 0.0)(category food) (grasp 29))
	;(item (type Objects) (name watermelon)(zone fridge)(image watermelon)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(heaviest yes))
	;(item (type Objects) (name sushi)(zone fridge)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	;(item (type Objects) (name melon)(zone fridge)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	;(item (type Objects) (name flakes)(zone fridge)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	;(item (type Objects) (name oat)(zone fridge)(image sushi)(attributes pick)(pose -3.55 -3.0 0.0)(category food))
	(item (type Objects) (name tomato_sauce)(zone dining_table)(image tomato_sauce)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(color red)(size 1) (wide 6) (height 2) (weight 9)(smallest yes))
	(item (type Objects) (name corn)(zone dining_table)(image corn)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(color yellow)(size 2) (wide 7) (height 3) (weight 10))
	(item (type Objects) (name apple)(zone dining_table)(image apple)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(color red)(size 3) (wide 8) (height 4) (weight 6))
	(item (type Objects) (name banana)(zone dining_table)(image banana)(attributes pick)(pose -3.55 -3.0 0.0)(category food)(color yellow)(size 9) (wide 1) (height 12) (weight 7))
	


	;;;; TOILETRIES
	;(item (type Objects) (name shampoo)(zone cupboard)(image shampoo)(attributes pick)(pose -3.55 -3.0 0.0) (category toiletries)(grasp 25))
	;(item (type Objects) (name soap)(zone cupboard)(image soap)(attributes pick)(pose -3.55 -3.0 0.0) (category toiletries)(grasp 28))
	;(item (type Objects) (name cloth)(zone cupboard)(image cloth)(attributes pick)(pose -3.55 -3.0 0.0) (category toiletries)(grasp 26))
	;(item (type Objects) (name sponge)(zone cupboard)(image sponge)(attributes pick)(pose -3.55 -3.0 0.0) (category toiletries)(grasp 27))

	;;;; CONTAINERS
	;(item (type Objects) (name bowl)(zone counter)(image bowl)(attributes pick)(pose -3.55 -3.0 0.0) (category containers)(biggest yes))
	;(item (type Objects) (name tray)(zone counter)(image tray)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))
	;(item (type Objects) (name plate)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))
	;(item (type Objects) (name mug)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))
	;(item (type Objects) (name glass)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))
	;(item (type Objects) (name box)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))
	;(item (type Objects) (name bag)(zone counter)(image plate)(attributes pick)(pose -3.55 -3.0 0.0) (category containers))


	;(item (type Objects) (name milk)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0)(category drinks)(grasp 33))
        ;(item (type Objects) (name juice)(zone sink)(image orange_juice)(attributes pick)(pose -3.55 -3.0 0.0)(category drinks) (grasp 32))
	(item (type Objects) (name soup) (zone sink) (image orange_juice)(attributes pick)(pose 0.0 0.0 0.0) (category food) (grasp 34))

        ;;;;;; Category 
        (item (type Category) (name snacks) (zone living_table)(quantity 4)(biggest pringles)(smallest chocolate) (heaviest pringles) (lightest chocolate))
        ;(item (type Category) (name candies) (zone center_table)(quantity 4) (biggest chocolate_bar) (smallest mints) (heaviest chocolate_bar) (lightest mints))
        (item (type Category) (name drinks) (zone dining_table)(quantity 4)(biggest milk) (smallest juice) (heaviest coke) (lightest milk))
        (item (type Category) (name food) (zone kitchen_table)(quantity 4)(biggest banana) (smallest tomato_sauce) (heaviest corn) (lightest apple))
        ;(item (type Category) (name toiletries) (zone cupboard)(quantity 4)(biggest shampoo) (smallest soap) (heaviest shampoo) (lightest cloth))
        ;(item (type Category) (name containers) (zone counter)(quantity 3)(biggest bowl) (smallest plate) (heaviest bowl) (lightest plate))

        (item (type Category) (name nil) (zone bed))
        (item (type Category) (name nil) (zone buro))
        (item (type Category) (name nil) (zone sofa))
        (item (type Category) (name nil) (zone tv_stand))
        (item (type Category) (name nil) (zone shelf))
        (item (type Category) (name nil) (zone fridge))
        (item (type Category) (name nil) (zone stove))
        ;(item (type Category) (name nil) (zone sideshelf))
        ;(item (type Category) (name nil) (zone bookcase))
        ;(item (type Category) (name nil) (zone dining_table))
        ;(item (type Category) (name nil) (zone cabinet))

        ;;;;; Property
        (item (type Property) (name biggest) (zone bed))
        (item (type Property) (name smallest) (zone bed))
        (item (type Property) (name heaviest) (zone bed))
        (item (type Property) (name lightest) (zone bed))
        (item (type Property) (name largest) (zone bed))
        (item (type Property) (name thinnest) (zone bed))

;;;;;;;;;;PERSONS

	;;;; Fameles
	;(item (type Objects) (name hanna)(zone living_room)(image hanna) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name barbara)(zone living_room)(image barbara) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name samantha)(zone living_room)(image samantha) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name erika)(zone living_room)(image erika) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name sophie)(zone living_room)(image sophie) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name jackie)(zone living_room)(image jackie) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name mary)(zone living_room)(image mary) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name patricia)(zone living_room)(image patricia) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name linda)(zone living_room)(image linda) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name barbara)(zone living_room)(image barbara) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name elizabeth)(zone living_room)(image elizabeth) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name jennifer)(zone living_room)(image jennifer) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name maria)(zone living_room)(image maria) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name susan)(zone living_room)(image susan) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name margaret)(zone living_room)(image margaret) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name dorothy)(zone living_room)(image dorothy) (attributes pick)(pose -1.87 8.64 0.0))

	;;; Males
	;(item (type Objects) (name ken)(zone living_room)(image ken) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name erik)(zone living_room)(image erik) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name samuel)(zone living_room)(image samuel) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name skyler)(zone kitchen)(image skyler) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name brian)(zone living_room)(image brian) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name thomas)(zone living_room)(image thomas) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name edward)(zone living_room)(image edward) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name michael)(zone living_room)(image michael) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name charlie)(zone living_room)(image charlie) (attributes pick)(pose -1.87 8.64 0.0))
	;(item (type Objects) (name alex)(zone living_room)(image alex) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name james)(zone living_room)(image james) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name john)(zone living_room)(image john) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name robert)(zone living_room)(image robert) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name william)(zone living_room)(image william) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name david)(zone living_room)(image david) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name richard)(zone living_room)(image richard) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name charles)(zone living_room)(image charles) (attributes pick)(pose -1.87 8.64 0.0))
	(item (type Objects) (name joseph)(zone living_room)(image joseph) (attributes pick)(pose -1.87 8.64 0.0))
	

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
	( item (type Room) (name bed_room) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))
	( item (type Room) (name living_room)(pose -1.87 8.64 0.0)(quantity 5) (quantitys 2))
	;( item (type Room) (name office) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 1))
	( item (type Room) (name kitchen)(pose -1.87 8.64 0.0)(quantity 8) (quantitys 2))
	( item (type Room) (name corridor)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 1))
	;( item (type Room) (name bathroom)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))
	( item (type Room) (name dinning_room)(pose -1.87 8.64 0.0)(quantity 1) (quantitys 0))

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

	(item (type Furniture) (name bed)(pose -3.55 -3.0 0.0)(possession bed_room)(attributes no_visited)(room bed_room))
	(item (type Furniture) (name buro)(pose -3.55 -3.0 0.0)(possession bed_room)(attributes no_visited)(room bed_room))
	(item (type Furniture) (name sofa)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name living_table)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name tv_stand)(pose -3.55 -3.0 0.0)(possession living_room)(attributes no_visited)(room living_room))
	(item (type Furniture) (name fridge)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name kitchen_table)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name stove)(pose -3.55 -3.0 0.0)(possession kitchen)(attributes no_visited)(room kitchen))
	(item (type Furniture) (name dining_table)(pose -3.55 -3.0 0.0)(possession dinning_room)(attributes no_visited)(room dinning_room))
	(item (type Furniture) (name shelf)(pose -3.55 -3.0 0.0)(possession dinning_room)(attributes no_visited)(room dinning_room))

        ;;;;;DOORS
        (item (type Door) (quantity 1) (room bed_room))
        (item (type Door) (quantity 2) (room living_room))
        ;(item (type Door) (quantity 3) (room office))
        (item (type Door) (quantity 2) (room kitchen))
        (item (type Door) (quantity 1) (room corridor))
        ;(item (type Door) (quantity 1) (room bathrooom))
        (item (type Door) (quantity 1) (room dinning_room))

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

