

(deffacts Initial-state-objects-rooms-zones-actors


;;;;;;;; LOCATIONS



(item (type Room) (name dinner_room) (pose -3.55 -3.0 0.0)(quantity 5) (quantitys 5))

(item (type Room) (name kitchen) (pose -3.55 -3.0 0.0)(quantity 1) (quantitys 1))

(item (type Room) (name living_room) (pose -3.55 -3.0 0.0)(quantity 2) (quantitys 2))



;;;;;;;;;;OBJECTS



(item (type Objects) (name chips) (zone desk) (image cereal) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room office)(grasp 2)(weight 1)(size 4)(height 1)(wide 10)(color yellow)(biggest yes)(smallest yes) (heaviest yes) (lightest yes))

(item (type Objects) (name pringles) (zone desk) (image cereal) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room office)(grasp 2)(weight 2)(size 5)(height 1)(wide 10)(color orange)(biggest yes)(smallest yes) (heaviest yes) (lightest yes))

(item (type Objects) (name senbei) (zone desk) (image cereal) (attributes pick) (pose 0.0 0.0 0.0) (category snacks) (room office)(grasp 2)(weight 2)(size 3)(height 1)(wide 10)(color brown)(biggest yes)(smallest yes) (heaviest yes) (lightest yes))



;;;;;;;;;;PEOPLE



(item (type Objects) (name angelica)(zone living_room)(image angelica)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name cintia)(zone living_room)(image cintia)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name jaime)(zone living_room)(image jaime)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name juan)(zone living_room)(image juan)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name octavio)(zone living_room)(image octavio)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name ramon)(zone living_room)(image ramon)(attributes pick)(pose -1.87 8.64 0.0))

(item (type Objects) (name rodrigo)(zone living_room)(image rodrigo)(attributes pick)(pose -1.87 8.64 0.0))



;;;;;;;;;;CATEGORYS



(item (type Category) (name drinks) (zone kitchen)(quantity 3)(biggest juice)(smallest orange_juice) (heaviest fruit_juice) (lightest grape_juice))  

(item (type Category) (name snacks) (zone kitchen)(quantity 3)(biggest juice)(smallest orange_juice) (heaviest fruit_juice) (lightest grape_juice))  

)