# code dependencies
import kb_services
import parsing
# network toolkit
import networkx as nx
# regular expressions 
import re


def diff(a, b):
    b = set(b)
    return [aa for aa in a if aa not in b]

def intersection(a,b):
	if isinstance(a, str):
		if " " in a:
			a = a.split(" ")
		else:
			a = [a]
	a = set(a)
	return [bb for bb in b if bb in a]

def set_mapping(mapping):
    global meaning_mapping_patterns, used_patterns
    if mapping == 'gpsr':
        meaning_mapping_patterns = meaning_mapping_patterns_gpsr
    elif mapping == 'eegpsr':
        meaning_mapping_patterns = meaning_mapping_patterns_eegpsr
    elif mapping == 'open_challenge': 
	meaning_mapping_patterns = meaning_mapping_patterns_open_challenge
    elif mapping == 'eegpsr2':
        meaning_mapping_patterns = meaning_mapping_patterns_eegpsr2
    elif mapping == 'restaurant':
        meaning_mapping_patterns = meaning_mapping_patterns_restaurant
    elif mapping == 'catering_comfort':
        meaning_mapping_patterns = meaning_mapping_patterns_catering_comfort
    elif mapping == 'receptionist':
        meaning_mapping_patterns = meaning_mapping_patterns_receptionist
    elif mapping == 'servingdrinks': 
        meaning_mapping_patterns = meaning_mapping_patterns_servingdrinks 
    elif mapping == 'spr': 
        meaning_mapping_patterns = meaning_mapping_patterns_spr
    elif mapping == 'where_is_this': 
        meaning_mapping_patterns = meaning_mapping_patterns_where_is_this
    used_patterns = [0]*len(meaning_mapping_patterns)

# 

# Patrones para OPEN CHALLENGE
meaning_mapping_patterns_open_challenge = [



	# test open challenge
	{"params": ["Action_get", "Object_find", "Action_deliver", "Destination_me"],
	"Action_get": [["take", "get"], ["vrb"], [], []],
	"Object_find": [[], ["noun"], ["item", "drink"], []],
	"Action_deliver": [["deliver"], ["vrb"], [], []],
	"Destination_me": [["john", "peter"], [], [], []],
	
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object_find- dining_room) (step )) " +
							"(task (plan user_speech) (action_type find_person_in_room) (params -Destination_me- dining_room) (step ))" + 
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	## Cubes Challenge
	{"params":["Action_put", "Object1", "Object2"],
	"Action_put": [["put"], ["vrb"], [], []],
	"Object1":[[], ["noun"], ["item"],[]],
	"Object2":[[],["noun"],["item"],[]],

	"conceptual_dependency":"(task (plan user_speech) (action_type explain_cubes_plan) (params -Object1- -Object2-) (step ))"+
	                        "(task (plan user_speech) (action_type update_object_location) (params location table) (step ))"+
                                "(task (plan user_speech) (action_type stack_state)(params ) (step ))" +
                                "(task (plan user_speech) (action_type speech_generator)(params speech_1)(step ))"+
                                "(task (plan user_speech) (action_type put_on_top) (params -Object1- -Object2-) (step))",
	
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}
	
]

meaning_mapping_patterns_gpsr = [


		# patrones para TMR 2017
	############################################# GetNDeliver

	# param: [["palabras", "clave"], ["noun", "vrb", "prep_phrase"], ["categoria", "item", "place", "person"], []]
		# take from and deliver to person
	#{"params": ["Action_get", "Get_object", "Source_get", "Action_deliver", "Destination_person", "Destination_location"],
	#"Action_get": [["get", "grasp", "take"], ["vrb"], [], []],
	#"Get_object": [[], ["noun"], ["item"], []],
	#"Source_get": [[], ["noun"], ["place"], []],
	#"Action_deliver": [["bring", "carry", "deliver", "take"], ["vrb"], [], []],
	#"Destination_person": [[], ["noun", "prep_phrase"], ["person"], []],
	#"Destination_location": [[], ["noun"], ["place"], []],
	#"conceptual_dependency": "(task (plan user_speech) (action_type update_object_location) (params -Get_object- -Source_get- ) (step 1)) " +
	#						"(task (plan user_speech) (action_type get_object) (params -Get_object- -Source_get-) (step 2)) " + 
	#						"(task (plan user_speech) (action_type find_person_in_room) (params -Destination_person- -Destination_location-) (step 3))" + 
	#						"(task (plan user_speech) (action_type handover_object) (params -Get_object-) (step 4))",
	#"verbal_confirmation": '',
	#"planner_confirmed": '',
	#"planner_not_confirmed": ''},
        
        #### add object to kdb
	{"params": ["Object", "Is", "Location"],
	"Object": [[], [], ["item"], []],
        "Is": [["is", "are"], [], [], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(inst (plan user_speech) (action_type set_object_location) (params -Object- -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ## obtain the object location
        {"params": ["Where", "Is", "Object"],
        "Where":[["where"],[],[],[]],
        "Is": [["is", "are"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(inst (plan user_speech) (action_type get_object_location) (params -Object-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#navigate
	{"params": ["Action_nav", "Location"],
	"Action_nav": [["navigate", "go", "locate", "enter"], [], [], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#question
	{"params": ["Action_talk", "Question"],
	"Action_talk": [["speak", "answer", "tell", "say"], [], [], []],
	"Question": [[], [], ["question"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type wait_for_user_instruction) (params question -Question-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#find person 1 parametro
	{"params": ["Action_find","Find_person"],
	"Action_find": [["find", "look_for", "locate"], [], [], []],
	"Find_person": [[], [], ["person"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Find_person-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#find person 2 parametros
	{"params": ["Find_person", "Person", "Person_location"],
	"Find_person": [["find", "locate", "look_for", "meet"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Person_location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Person_location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	# deliver to me
	{"params": ["Action_deliver", "Person"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"Person": [["me"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	# deliver to person
	{"params": ["Action_deliver", "Person"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"Person": [[], [], ["person"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))" +
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	# deliver to person 2 parametros
	{"params": ["Action_deliver", "Person", "Location"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" +
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#grasp object 2 parametros
	{"params": ["Action_get", "Get_object", "Source_get"],
	"Action_get": [["get", "grasp", "take"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"Source_get": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Get_object- -Source_get-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#grasp object 1 parametro
	{"params": ["Action_get", "Get_object"],
	"Action_get": [["get", "grasp", "take", "look_for"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Get_object-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#deliver in place
	{"params": ["Action_place", "Destination_place"],
	"Action_place": [["place", "deliver", "put"], [], [], []],
	"Destination_place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type deliver_in_position) (params -Destination_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#follow 1 parametro
	{"params": ["Action_follow", "Pron"],
	"Action_follow": [["follow", "after"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#follow to room
	{"params": ["Action_follow", "Pron", "Location"],
	"Action_follow": [["follow", "after"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type follow_man) (params man -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#guide to room
	{"params": ["Action_guide", "Pron", "Location"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#guide to room 2 parameters (was comented for Sidney gpsr)
	#{"params": ["Action_guide", "Person", "Location"],
	#"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	#"Person": [[], [], ["person"], []],
	#"Location":[[], [], ["place"], []],
	#"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
	#"verbal_confirmation": '',
	#"planner_confirmed": '',
	#"planner_not_confirmed": ''},


	#####################NAGOYA 2017
	##########################
	####################### MANIPULATION

	#$take to the {placement 2}
	{"params": ["Action_take", "Object", "To", "Place"],
	"Action_take": [[ "get" , "grasp" , "take" , "pick up", "bring", "place"], [], [], []],
	"Object": [[], [], ["item"], []],
	"To": [["to"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$vbplace the $object on the {placement 2}
	{"params": ["Action_take", "Object", "Place"],
	"Action_take": [["put", "place", "deliver"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$deliver  = $vbbring me the $object
	{"params": ["Action_take", "Person", "Object"],
	"Action_take": [["bring", "give", "deliver"], [], [], []],
	"Person": [["me"], [], [], []],
	"Object": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$vbdeliver the $object to $someone
	{"params": ["Action_take", "Object", "Person", "Location"],
	"Action_take": [["bring", "give", "deliver"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Person": [[], [], ["person"], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$takefrom to the {placement 2}
	{"params": ["Action_take", "Object","Place_first","To", "Place_second"],
	"Action_take": [["get" , "grasp" , "take" , "pick up", "bring"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Place_first": [[], [], ["place", "room"], []],
	"To": [["to"], [], [], []],
	"Place_second": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Place_first-) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place_second-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$goplace, $vbfind the $object, and ($delivme | $delivat)
	#$goplace, $vbfind the $object, and $place

	#$vbfind the $object 1 parametro
	{"params": ["Action_get", "Get_object"],
	"Action_get": [["find", "look_for", "locate"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Get_object-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$vbbring (me | to $whowhere) the {kobject} from the {placement}
	{"params": ["Action_take", "Person", "Place_first", "Object", "Place_second"],
	"Action_take": [["bring" , "give"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Place_first": [[], [], ["place", "room"], []],
	"Object": [[], [], ["item"], []],
	"Place_second": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Place_second-) (step )) " +
				"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Place_first-) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	{"params": ["Action_take", "Person", "Object", "Place_second"],
	"Action_take": [["bring" , "give"], [], [], []],
	"Person": [["me"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Place_second": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Place_second-) (step )) " +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	###### FIND PEOPLE

	#$talk to $whowhere    $findp in the {room} and $talk
	{"params": ["Action_talk", "Question", "Person", "Location"],
	"Action_talk": [["speak", "answer", "tell", "say"], [], [], []],
	"Question": [[], [], ["question"], []],
	"Person": [[], [], ["person"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" + 
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question -Question-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#Tell me the name of the person at the {beacon}  Tell me the name of the person in the {room}
	{"params": ["Action_talk","Me", "Name", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Me": [["me"], [], [], []],
	"Name":[["name"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params person -Location-) (step ))" + 
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question ask_name) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_name) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$findp in the {room}
	{"params": ["Action_get", "Person", "Location"],
	"Action_get": [["find", "look_for", "locate"], [], [], []],
	"Person": [["person", "someone"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params person -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#### FIND OBJECTS
	#$fndobj   = $vbfind the {kobject?} in the {room}
	{"params": ["Action_get", "Object", "Location"],
	"Action_get": [["find", "look_for", "locate"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))"+
				"(task (plan user_speech) (action_type find_object_in_room) (params -Object- -Location-) (step ))",
	"verbal_confirmation": '',	
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$fndobj   = Tell me how many $object there are on the {placement}
	{"params": ["Action_talk", "Person", "Many", "Object", "Location_second"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Object": [[], [], ["item"], []],
	"Location_second":[[],[],["place", "room"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_objects) (params -Object- -Location_second- nil) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	######### NAGOYA 2017 CATEGORY 2

	#$vbdeliver the $object to $someone
	{"params": ["Action_take", "Object", "Gesture", "Location"],
	"Action_take": [["bring", "give", "deliver"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Gesture":[["waving", "rising_their_left_arm", "raising_their_right_arm", "pointing_to_the_left", "pointing_to_the_right"],[],[],[]],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_pgg_person) (params -Gesture- -Location-) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	###### FIND PEOPLE

	#$fndppl   = $talk to $whowhere
	{"params": ["Action_talk", "Question", "Gesture", "Location"],
	"Action_talk": [["speak", "answer", "tell", "say"], [], [], []],
	"Question": [[], [], ["question"], []],
	"Gesture":[[],[],["gesture"],[]],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_pgg_person) (params -Gesture- -Location-) (step ))" + 
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question -Question-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$findp = $vbfind a ($pgesture | $ppose) person
	{"params": ["Action_find","Person", "PGG"],
	"Action_find": [["find", "locate", "look_for"], [], [], []],
        "Person": [["man"],[],[],[]],
	"PGG":[[],[],["gprsn", "posprs", "gesture"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_pgg_person) (params -PGG- place_loc) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$findp = $vbfind a $pgenders
	{"params": ["Action_find","Person"],
	"Action_find": [["find", "locate", "look_for"], [], [], []],
        "Person": [["man"],[],[],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_pgg_person) (params -Person- place_loc) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$findppl = $findp in the {room}
	{"params": ["Action_find", "PGG", "Location"],
	"Action_find": [["find", "locate", "look_for"], [], [], []],
	"PGG":[["man"],[],[],[]],
	"Location":[[],[],["place", "room"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_pgg_person) (params -PGG- -Location-) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$findppl = $findp in the {room}
	{"params": ["Action_find", "Person", "PGG", "Location"],
	"Action_find": [["find", "locate", "look_for"], [], [], []],
        "Person": [["man"],[],[],[]],
	"PGG":[[],[],["gprsn", "posprs", "gesture"],[]],
	"Location":[[],[],["place", "room"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_pgg_person) (params -PGG- -Location-) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$fndppl   = Tell me the ( gender | pose) of the person at the {beacon}
	#$fndppl   = Tell me the ( gender | pose) of the person in the {room}
	{"params": ["Action_talk","Me", "Genderpose", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Me": [["me"], [], [], []],
	"Genderpose":[["gender", "pose"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_gender_pose_person) (params -Genderpose- -Location-) (step ))" + 
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_gender_pose) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$fndppl   = tell me how many people in the {room} are ($pgenderp | $pose)  
	{"params": ["Action_talk", "Person", "How", "Many", "People", "Location", "Pgender"],
	"Action_talk": [["tell"], [], [], []],
	"Person": [["me"],[],[],[]],
	"How": [["how"],[],[],[]],
	"Many": [["meany"],[],[],[]],
	"People": [["people"],[],[],[]],
	"Location":[[], [], ["place", "room"], []],
	"Pgender":[["men", "women", "boys", "girls", "male", "female", "sitting", "standing", "lying"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_gender_pose_crowd) (params -Pgender- -Location-) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_how_many_people) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	

	#Tell me how many {category} there are on the {placement}
	{"params": ["Action_talk", "Person", "Many", "Category", "Location_second"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Category": [["snacks", "cutlery", "food", "drinks", "tableware", "containers", "fruits", "cleaning_stuff"], [], [], []],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_category) (params -Category- -Location_second- nil) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$fndobj   = Tell me what's the $oprop object on the {placement}
	{"params": ["Action_talk", "Person", "Property", "Object", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person": [["me"], [], [], []],
	"Property": [["biggest", "smallest", "heaviest", "lightest", "largest", "thinnest"], [], [], []],
	"Object": [["object"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_prop_object) (params -Property- nil) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_what) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$fndobj   = Tell me what's the $oprop {category} on the {placement}
	{"params": ["Action_talk", "Person", "Property", "Category", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person": [["me"],[],[],[]],
	"Property": [["biggest", "smallest", "heaviest", "lightest", "largest", "thinnest"], [], [], []],
	"Category": [["snacks", "cutlery", "food", "drinks", "tableware", "containers", "fruits", "cleaning_stuff"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_prop_object) (params -Property- -Category-) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_what_cat) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$fndobj   = $vbfind the {category} in the {room}
	{"params": ["Action_get", "Category", "Location"],
	"Action_get": [["find", "look_for", "locate"], [], [], []],
	"Category": [["snacks", "cutlery", "food", "drinks", "tableware", "containers", "fruits", "cleaning_stuff"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_category_room) (params -Category- -Location-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$follow   = $vbfollow {name 1} from the {beacon 1} to the {room 2}
	#$follow   = meet {name 1} at the {beacon 1} and $vbfollow them $fllwdest
	{"params": ["Action_follow", "Person", "Location_first", "Location_second"],
	"Action_follow": [["follow", "after"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_first-) (step )) " +
				"(task (plan user_speech) (action_type follow_man) (params man -Location_second-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$gdcmd    = $vbguide {name 1} from the {beacon 1} to the {beacon 2}
	#$gdcmd    = meet {name 1} at the {beacon 1} and $guideto
	{"params": ["Action_guide", "Person", "Location_first", "Location_second"],
	"Action_guide": [["guide" , "escort" , "take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_first-) (step )) " +
				"(task (plan user_speech) (action_type get_object) (params man_guide -Location_second-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$gdcmd    = $vbguide {name 1} to the {beacon 2}, $gdwhere
	{"params": ["Action_guide", "Person", "Location_first", "May", "Action_find", "Location_second"],
	"Action_guide": [["guide" , "escort" , "take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"May":[["may", "can", "will"],[],[],[]],
	"Action_find":[["find"],[],[],[]],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_second-) (step )) " +
				"(task (plan user_speech) (action_type get_object) (params man_guide -Location_first-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
	#join the dark side
        {"params": ["Join", "Dark", "Side"],
         "Join": [["join"], [], [], []],       
         "Dark": [["dark"], [], [], []],       
         "Side": [["side"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type speech_generator) (params join_dark_side) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        ########### SIDNEY 2019
        
        ###
        ### MANIPULATION
        ###
        
        ##$vbbtake my $luggage to the $taxi
	{"params": ["Action_take", "Luggage","Taxi"],
	"Action_take": [["bring", "take"], [], [], []],
	"Luggage": [["bag", "baggage", "valise", "suitcase", "trolley"], [], [], []],
	"Taxi": [["uber", "taxi", "cab"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_bag) (params -Luggage-) (step )) " +
                                "(task (plan user_speech) (action_type follow_to_taxi) (params man no_location)(step ))" +
                                "(task (plan user_speech) (action_type handover_object) (params bag)(step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ###############
        ### COMPLEX MANIPULATION
        ############
        
        ##$vbbring me the $abspos object $cmanobjsrc
	{"params": ["Action_take", "Person", "Abspos", "Object", "Place"],
	"Action_take": [["give", "bring"], [], [], []],
	"Person": [["me"], [], [], []],
	"Abspos": [[], [], ["abspos"], []],
	"Object": [["object"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object)(params -Object- -Place- -Abspos-)(step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params )(step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ##$vbbring me the object $relpos the {object}  $cmanobjsrc
	{"params": ["Action_take", "Person", "Relpos",  "Object", "Place"],
	"Action_take": [["give", "bring"], [], [], []],
	"Person": [["me"], [], [], []],
	"Relpos": [[], [], ["relpos"], []],
	"Object": [[], [], ["item"], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params object -Place- -Relpos- -Object-) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ##$vbbring me the $oprop (object | {category}) $cmanobjsrc
	{"params": ["Action_take", "Person", "Oprop",  "Object", "Place"],
	"Action_take": [["give", "bring"], [], [], []],
	"Person": [["me"], [], [], []],
	"Oprop": [[], [], ["adjectivea"], []],
	"Object": [["object"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type get_object) (params object -Place- -Oprop- nil) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##$vbbring me the $oprop (object | {category}) $cmanobjsrc
	{"params": ["Action_take", "Person", "Oprop",  "Category", "Place"],
	"Action_take": [["give", "bring"], [], [], []],
	"Person": [["me"], [], [], []],
	"Oprop": [[], [], ["adjectivea"], []],
	"Category": [[], [], ["category"], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params object -Place- -Oprop- -Category-) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##$cmancmd     = $vbclean the {room meta: Place 3 objects randomly, at least one on the floor.}
	{"params": ["Action_clean", "Place"],
	"Action_clean": [["clean", "tidy", "neaten", "orer"], [], [], []],
	"Place": [[], [], ["room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type clean_up) (params -Place-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##$cmancmd     = $vbtakeout the $garbage
	{"params": ["Action_take", "Garbage"],
	"Action_take": [["take", "dump"], [], [], []],
	"Garbage": [["litter","garbage", "trash", "waste", "debris", "junk"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type take_out_garbage) (params -Garbage-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        

        ######
        ### Find Objects
        ######
        
        ##tell me which are the three $oprop (objects | {category}) on the {placement 1 meta: Place at least 5 objects on the {placement 1}}
	{"params": ["Action_tell", "Person", "Three", "Oprop", "Object", "Location"],
	"Action_tell": [["tell"], [], [], []],
	"Person": [["me"], [], [], []],
        "Three": [["three", "3"],[],[],[]],
	"Oprop": [[], [], ["adjectivea"], []],
	"Object": [["object", "objects"], [], [], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_prop_object) (params -Oprop- nil three) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_what_three_cat) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##tell me which are the three $oprop (objects | {category}) on the {placement 1 meta: Place at least 5 objects on the {placement 1}}
	{"params": ["Action_tell", "Person", "Three", "Oprop", "Category", "Location"],
	"Action_tell": [["tell"], [], [], []],
	"Person": [["me"], [], [], []],
        "Three": [["three", "3"],[],[],[]],
	"Oprop": [[], [], ["adjectivea"], []],
	"Category": [[], [], ["category"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_prop_object) (params -Oprop- -Category- three) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_what_three_cat) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ##$vbfind three {category} in the {room meta: Place only 3 objects in the room}
	{"params": ["Action_find", "Three", "Category", "Location"],
        "Action_find": [["find", "look_for", "locate"], [], [], []],
	"Three": [["three", "3"], [], [], []],
	"Category": [[], [], ["category"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))" +
				"(task (plan user_speech) (action_type find_category_room) (params -Category- -Location- three) (step )) ", 
	"verbal_confirmation": '',	
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        #guide to room
	{"params": ["Action_guide", "Pron", "Back"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Back":[["back"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide current_loc) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ########
        ### PARTY HOST
        #########
        
        #$vbserve (drinks | snacks) to $phpeopler
	{"params": ["Action_serve", "EatDrink", "Phpeople", "Place"],
	"Action_serve": [["serve" , "arrange" ,"deliver" , "distribute" , "give", "provide"], [], [], []],
	"EatDrink":[[],[],["category"],[]],
	"Phpeople":[["everyone", "people", "men", "women", "guests", "elders", "children"],[],[],[]],
	"Place":[[],[],["place", "room"],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type offer_eat_drink) (params -Phpeople- -EatDrink- -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$vbmeet {name} at the $door and introduce {pron} to $phpeopler
	{"params": ["Action_meet", "Person", "Pos", "Door"],
	"Action_meet": [["contact" , "face" ,"find" , "greet"], [], [], []],
	"Person":[[],[],["person"],[]],
	"Pos":[["front", "back", "main", "rear"],[],[],[]],
	"Door":[["entrance", "door"],[],[],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_door) (params -Person- -Pos-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
	{"params": ["Action_introduce", "Pron", "Phpeople", "Place"],
	"Action_introduce": [["introduce"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Phpeople":[["everyone", "people", "men", "women", "guests", "elders", "children"],[],[],[]],
	"Place":[[],[],["place", "room"],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type introduce_people) (params -Phpeople- -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$vbmeet {name} at the {beacon} and ask {pron} to leave
	{"params": ["Action_meet", "Person", "Place"],
	"Action_meet": [["contact" , "face" ,"find" , "greet"], [], [], []],
	"Person":[[],[],["person"],[]],
	"Place":[[],[],["place", "room"],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        {"params": ["Action_ask", "Pron", "To", "Leave"],
	"Action_ask": [["ask"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"To":[["to"],[],[],[]],
	"Leave":[["leave"],[],[],[]],
	#"conceptual_dependency": "(task (plan user_speech) (action_type make_question) (params leave) (step ))",
	"conceptual_dependency": "(task (plan user_speech) (action_type guide_to_taxi) (params man_guide ask_for_leave) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$vbmeet {name 1} at the {beacon 1} and introduce {pron} to {name 2} at the {beacon 2}
	{"params": ["Action_introduce", "Pron", "Person", "Place"],
	"Action_introduce": [["introduce"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Person":[[],[],["person"],[]],
	"Place":[[],[],["place", "room"],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type introduce_person) (params -Person- -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$vbmeet {name 1} at the {beacon 1} and $vbguide {pron} to {pron pos} $taxi
	{"params": ["Action_guide", "Pron", "To", "Taxi"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"To":[["To"],[],[],[]],
	"Taxi":[["taxi", "cab", "uber"],[],[],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type guide_to_taxi) (params man_guide no_ask_for_leave) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	######
	## INCOMPLETE
	####
	
	#$incomplete = $vbfollow {name 1 meta: {name 1} is at the {beacon 1}}
	{"params": ["Action_follow", "Person"],
	 "Action_follow": [["follow", "after"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person- ) (step ))" +
				 "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},
	
	#$incomplete = $cmanwarn $vbbring me (a | some) {object?}
	{"params": ["Action_bring", "Me", "Category"],
	 "Action_bring": [["bring", "give", "deliver"],[],[],[]],
	 "Me": [["me"],[],[],[]],
	 "Category": [[],[],["category"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
				 "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				 "(task (plan user_speech) (action_type handover_object) (params )(step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},
	
	#$incomplete = $cmanwarn $vbdeliver {object?} to a {gesture 1} person {void meta: }
	{"params": ["Action_deliver", "Category", "Gesture", "Place"],
	 "Action_deliver": [["bring", "give", "deliver"],[],[],[]],
	 "Category": [[],[],["category"],[]],
	 "Gesture": [[],[],["gesture"],[]],
	 "Place": [[],[],["place", "room"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
                                 "(task (plan user_speech) (action_type update_object_location)(params location -Place- )(step ))" +
				 "(task (plan user_speech) (action_type find_pgg_person) (params -Gesture- -Place-)(step ))" +
				 "(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},
	
	#$incomplete = $vbguide {name 1 meta: {name 1} is at the {beacon 1} to the {beacon 2}}
	{"params": ["Action_guide", "Person", "Place"],
	 "Action_guide": [["guide", "take", "escort", "lead", "accompany"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "Place": [[],[],["place"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params man_guide -Place-) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},
	 
        #incomplete = meet $inguidewho and $vbguide {pron}
	{"params": ["Action_find", "Person"],
	 "Action_find": [["meet"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
                                 "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '', 
	 "planner_not_confirmed": ''},
	
        #and $vbguide {pron}	
	{"params": ["Action_guide", "Pron"],
	 "Action_guide": [["guide", "escort", "take", "lead", "accompany"],[],[],[]],
	 "Pron": [["her", "him", "it"],[],[],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question place_destiny person) (step ))" +
                                 "(task (plan user_speech) (action_type get_object) (params man_guide) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''}, 

	####
	### OFFER objects
	####
	{"params": ["My", "Person_name"],
	"My": [["My", "name"], [], [], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#I am
	{"params": ["I", "Person_name"],
	"I": [["i"], [], ["person"], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #I want drink
	{"params": ["Person", "Want", "Favorite_drink"],
	"Person": [["i"], [], [], []],
	"Want": [["want"], [], [], []],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "-Favorite_drink-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #This is the object
        {"params": ["This", "Is", "Object"],
	"This": [["this"], [], [], []],
	"Is": [["is"], [], [], []],
	"Object": [[], [], ["item"], []],
	"conceptual_dependency": "-Object-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #This is the unknown
        {"params": ["This", "Is", "Unknown", "Object"],
	"This": [["this"], [], [], []],
	"Is": [["is"], [], [], []],
	"Unknown": [["unknown"], [], [], []],
        "Object": [["object"],[],[],[]],
	"conceptual_dependency": "unknown_object",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	####
	### GET LOCATION NAMES
	###
	{"params": ["At", "Place"],
	"At": [["at", "in"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "-Place-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}
]


meaning_mapping_patterns_spr = [
        #$Where is the {placement} located?
        #$Where is the {beacon} located?
	
         {"params": ["Action_talk", "Verb_first", "Location","Verb_second"],
         "Action_talk": [["where"], [], [], []],
         "Verb_first": [["is"], ["vrb"], [], []],
         "Location": [[], [], ["place"], []],
         "Verb_second": [["located"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_what_place) (params -Location-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},
        
        #$In which room is the {placement}
        #$In which room is the {beacon}?
        {"params": ["PrepLoc", "Action_talk", "Location_first", "Location_second" ],
         "PrepLoc": [["in"], ["prep_loc"], [], []],
         "Action_talk": [["wich"], [], [], []],
         "Location_first": [[], [], ["place"], []], 
         "Location_second": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_what_place) (params -Location_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$How many doors does the {room} have?
        {"params": ["Action_talk", "Many", "Doors", "Location"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "Doors": [["doors"], [], [], []],
         "Location": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_doors) (params -Location-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$How many ({placement} | {beacon}) are in the {room}?
        {"params": ["Action_talk", "Many", "Location_first", "Location_second"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "Location_first": [[], [], ["place"], []],
         "Location_second": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_for) (params -Location_first- -Location_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Where can I find the {object}?
        {"params": ["Action_talk", "Action_find", "Object"],
         "Action_talk": [["where"], [], [], []],
         "Action_find": [["find"], ["vrb"], [], []],
         "Object": [[], [], ["item"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_where) (params -Object-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Between the {object 1} and {object 2}, which one is $adjr?
        {"params": ["Action_talk", "Object_first", "Object_second", "Action_compare", "Adjective"],
         "Action_talk": [["Between"], [], [], []],
         "Object_first": [[], [], ["item"], []],
         "Object_second": [[], [], ["item"], []],
         "Action_compare": [["wich"], [], [], []],
         "Adjective": [[], [], ["adjectiver"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_compare) (params -Adjective- -Object_first- -Object_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        #$How many {category} are there?
        {"params": ["Action_talk", "Many", "Category", "Vrb"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "Category": [[], [], ["category"], []],
         "Vrb": [["are"], ["vrb"], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_cat) (params -Category-) (step )) ",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        #$What's the color of the {kobject}?
        {"params": ["Action_talk", "Adjective_colour", "Object"],
         "Action_talk": [["what's"], [], [], []],
         "Adjective_colour": [["color"], [], [], []],
         "Object": [[], [], ["item"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_color) (params -Object-) (step )) ",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        #$How many ({category} | objects) are in the {placement}?
        {"params": ["Action_talk", "Many", "Category", "Location"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "Category": [[], [], ["category"], []],
         "Location": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_cat_place) (params -Category- -Location-) (step )) ",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        {"params": ["Action_talk", "Many", "Objects", "Location"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "Objects": [["objects"], [], [], []],
         "Location": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_obj_place) (params -Location-) (step )) ",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        #$What objects are stored in the {placement}?
        {"params": ["Action_talk", "Subject", "Vrb", "Location"],
         "Action_talk": [["what"], [], [], []],
         "Subject": [["objects"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Location": [[], [], ["place"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_what_obj) (params -Location-) (step )) ",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''
        },

        #$Where can I find a ({object} | {category})?
        {"params": ["Action_talk", "Action_find", "Category"],
         "Action_talk": [["where"], [], [], []],
         "Action_find": [["find"], ["vrb"], [], []],
         "Category": [[], [], ["category"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_where) (params -Category-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$what is the category of the {object}?
        {"params": ["Action_talk", "Vrb", "Subject", "Object"],
         "Action_talk": [["what"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Subject": [["category"], [], [], []],
         "Object": [[], [], ["item"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_category) (params -Object-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Do the {object 1} and {object 2} belong to the same category?
        {"params": ["Action_talk", "Object_first", "Object_second", "Verb", "Preposition", "Adjective", "Category"],
         "Action_talk": [["do"], [], [], []],
         "Object_first": [[], [], ["item"], []],
         "Object_second": [[], [], ["item"], []],
         "Verb": [["belong"], [], [], []],
         "Preposition": [["to"], [], [], []],
         "Adjective": [["same"], [], [], []],
         "Category": [["Category"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_same_category) (params -Object_first- -Object_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Which is the $adja ({category} | object)?
        {"params": ["Action_talk", "Vrb", "Adjective", "Category"],
         "Action_talk": [["wich"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Adjective": [[], [], ["adjectivea"], []],
         "Category": [[], [], ["category"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_absolute_compare) (params -Category- -Adjective-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},
        {"params": ["Action_talk", "Vrb", "Adjective", "Object"],
         "Action_talk": [["wich"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Adjective": [[], [], ["adjectivea"], []],
         "Object": [["object"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_absolute_compare) (params -Adjective-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$How many $people are in the crowd?
        {"params": ["Action_talk", "Many", "People", "Vrb", "Prep_loc", "Crowd"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "People": [[], [], ["people"], []],
         "Prep_loc": [["in"], ["prep_loc"], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Crowd": [["crowd"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people) (params -People-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$How many people in the crowd are ($posppl | {gesture})?
        {"params": ["Action_talk", "Many", "People", "Prep_loc", "Crowd",  "Vrb", "Posprs"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "People": [["people"], [], [], []],
         "Prep_loc": [["in"], ["prep_loc"], [], []],
         "Crowd": [["crowd"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Posprs": [[], [], ["posprs"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_posprs) (params -Posprs-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},
        {"params": ["Action_talk", "Many", "People", "Prep_loc", "Crowd",  "Vrb", "Posprs_first", "Conjunction", "Posprs_second"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "People": [["people"], [], [], []],
         "Prep_loc": [["in"], ["prep_loc"], [], []],
         "Crowd": [["crowd"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Posprs_first": [[], [], ["posprs"], []],
         "Conjunction": [["or"], [], [], []],
         "Posprs_second": [[], [], ["posprs"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_posprs) (params -Posprs_first- -Posprs_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},
        {"params": ["Action_talk", "Many", "People", "Prep_loc", "Crowd",  "Vrb", "Gesture"],
         "Action_talk": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "People": [["people"], [], [], []],
         "Prep_loc": [["in"], ["prep_loc"], [], []],
         "Crowd": [["crowd"], [], [], []],
         "Vrb": [["is"], ["vrb"], [], []],
         "Gesture": [[], [], ["gesture"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_gesture) (params -Gesture-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Tell me the number of $people in the crowd
        {"params": ["Vrb", "Me", "Det", "Number", "Rel_pro",  "People", "Prep_loc", "Det", "Crowd"],
         "Vrb": [["tell"], ["vrb"], [], []],
         "Me": [["me"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Number": [["number"], [], [], []],
         "Rel_pro": [["of"], ["rel_pro"], [], []],
         "People": [[], [], ["people"], []],
         "Prep_loc": [["in"], ["prep_loc"], [], []],
         "Crowd": [["crowd"], [], [], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people) (params -People-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Was the person $posprs a $gprsng?
        {"params": ["Action_talk", "Det", "Noun", "Posprs", "Gprsn_first",  "Conjunction", "Gprsn_second"],
         "Action_talk": [["was"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Noun": [["man"], ["noun"], [], []],
         "Posprs": [[], [], ["posprs"], []],
         "Gprsn_first": [[], [], ["gprsn"], []],
         "Conjunction": [["or"], [], [], []],
         "Gprsn_second": [[], [], ["gprsn"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_was) (params -Posprs- -Gprsn_first- -Gprsn_second-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Tell me if the person ($posprs | {gesture}) was a $gprsn?
        {"params": ["Vrb_first", "Me", "Condition", "Det", "Noun",  "Posprs", "Vrb_second", "Existencial", "Gprsn"],
         "Vrb_first": [["tell"], ["vrb"], [], []],
         "Me": [["me"], [], [], []],
         "Condition": [["if"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Noun": [["man"], [], [], []],
         "Posprs": [[], [], ["posprs"], []],
         "Vrb_second": [["was"], [], [], []],
         "Existencial": [["a"], ["existencial"], [], []],
         "Gprsn": [[], [], ["gprsn"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_was) (params -Posprs- -Gprsn-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Tell me if the person ($posprs | {gesture}) was a $gprsn?
        {"params": ["Vrb_first", "Me", "Condition", "Det",  "Posprs", "Vrb_second", "Existencial", "Gprsn"],
         "Vrb_first": [["tell"], ["vrb"], [], []],
         "Me": [["me"], [], [], []],
         "Condition": [["if"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Posprs": [[], [], ["posprs"], []],
         "Vrb_second": [["was"], [], [], []],
         "Existencial": [["a"], ["existencial"], [], []],
         "Gprsn": [[], [], ["gprsn"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_was) (params -Posprs- -Gprsn-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        {"params": ["Vrb_first", "Me", "Condition", "Det", "Gesture",  "Noun", "Vrb_second", "Existencial", "Gprsn"],
         "Vrb_first": [["tell"], ["vrb"], [], []],
         "Me": [["me"], [], [], []],
         "Condition": [["if"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Gesture": [[], [], ["gesture"], []],
         "Noun": [["man"], [], [], []],
         "Vrb_second": [["was"], [], [], []],
         "Existencial": [["a"], ["existencial"], [], []],
         "Gprsn": [[], [], ["gprsn"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_was_gesture) (params -Gesture- -Gprsn-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},
        
        {"params": ["Vrb_first", "Me", "Condition", "Det", "Gesture", "Vrb_second", "Existencial", "Gprsn"],
         "Vrb_first": [["tell"], ["vrb"], [], []],
         "Me": [["me"], [], [], []],
         "Condition": [["if"], [], [], []],
         "Det": [["the"], ["det"], [], []],
         "Gesture": [[], [], ["gesture"], []],
         "Vrb_second": [["was"], [], [], []],
         "Existencial": [["a"], ["existencial"], [], []],
         "Gprsn": [[], [], ["gprsn"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_was_gesture) (params -Gesture- -Gprsn-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''},

        #$Tell me how many people were wearing $color
        {"params": ["Vrb_first", "Pronoun", "How", "Many", "People", "Vrb_second", "Vrb_third", "Color"],
         "Vrb_first": [["tell"], ["vrb"], [], []],       
         "Pronoun": [["me"], [], [], []],       
         "How": [["how"], [], [], []],
         "Many": [["many"], [], [], []],
         "People": [["people"], [], [], []],
         "Vrb_second": [["were"], [], [], []],
         "Vrb_third": [["wearing"], [], [], []],
         "Color": [[], [], ["color"], []],
	 "conceptual_dependency": "(task (plan user_speech) (action_type cmd_many_people_color) (params -Color-) (step )) ", 
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
         "planner_not_confirmed": ''}

]

meaning_mapping_patterns_eegpsr = [
	#category 5 eegpsr NAGOYA
	
	#$incomplete = $vbfollow {name 1 meta: {name 1} is at the {beacon 1}}
	{"params": ["Action_follow", "Person"],
	 "Action_follow": [["follow", "after"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person- ) (step ))" +
				 "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},

	#$incomplete = $cmanwarn $vbbring me (a | some) {object?}
	{"params": ["Action_bring", "Me", "Category"],
	 "Action_bring": [["bring", "give", "deliver"],[],[],[]],
	 "Me": [["me"],[],[],[]],
	 "Category": [[],[],["category"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
				 "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				 "(task (plan user_speech) (action_type handover_object) (params )(step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},

	#$incomplete = $cmanwarn $vbdeliver {object?} to me
	#{"params": ["Action_deliver", "Category", "Me"],
	# "Action_deliver": [["bring", "give", "deliver"],[],[],[]],
	# "Category": [[],[],["category"],[]],
	# "Me": [["me"],[],[],[]],
	# "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params object) (step ))" +
	#			 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
	#			 "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
	#			 "(task (plan user_speech) (action_type handover_object) (params )(step ))",
	# "verbal_confirmation": '',
	# "planner_confirmed": '',
	# "planner_not_confirmed": ''},
	
	#$incomplete = $cmanwarn $vbdeliver {object?} to {name 1 meta: {name 1} is at the {beacon 1}}
	{"params": ["Action_deliver", "Category", "Person"],
	 "Action_deliver": [["bring", "give", "deliver"],[],[],[]],
	 "Category": [[],[],["category"],[]],
	 "Person": [[],[],["person"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				 "(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person-)(step ))" +
				 "(task (plan user_speech) (action_type handover_object) (params )(step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},

	#$incomplete = $cmanwarn $vbdeliver {object?} to a {gesture 1} person {void meta: }
	{"params": ["Action_deliver", "Category", "Gesture"],
	 "Action_deliver": [["bring", "give", "deliver"],[],[],[]],
	 "Category": [[],[],["category"],[]],
	 "Gesture": [[],[],["gesture"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				 "(task (plan user_speech) (action_type ask_info) (params question gesture_place_origin -Gesture-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params default_location ) (step ))" +
                                 "(task (plan user_speech) (action_type update_object_location)(params temp place_loc )(step ))" +
				 "(task (plan user_speech) (action_type find_pgg_person) (params -Gesture- place_loc)(step ))" +
				 "(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},
	
	#$incomplete = $vbguide {name 1 meta: {name 1} is at the {beacon 1} to the {beacon 2}}
	{"params": ["Action_guide", "Person", "Location"],
	 "Action_guide": [["guide", "take", "escort", "lead", "accompany"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "Location": [[],[],["place"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))" +
				 "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},

	 #incomplete = meet $inguidewho and $vbguide {pron}
	{"params": ["Action_find", "Person"],
	 "Action_find": [["meet"],[],[],[]],
	 "Person": [[],[],["person"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
				 "(task (plan user_speech) (action_type ask_info) (params question place_destiny -Person-) (step ))" +
				 "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '', 
	 "planner_not_confirmed": ''},

	
	#$incomplete  = $gobeacon
	{"params": ["Action_go", "Location"],
	 "Action_go": [["navigate", "go", "enter"],[],[],[]],
	 "Location": [[],[],["place"],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question place_destiny person) (step ))"
				"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''},

	#and $vbguide {pron}	
	{"params": ["Action_guide", "Pron"],
	 "Action_guide": [["guide", "escort", "take", "lead", "accompany"],[],[],[]],
	 "Pron": [["her", "him", "it"],[],[],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type get_object) (params man_guide) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''} 

]

meaning_mapping_patterns_eegpsr2 = [
        #category 2 eegpsr Montreal
	
        ###############################
        ### single commands         ###
        ###############################

        #navigate
	{"params": ["Action_nav", "Location"],
	"Action_nav": [["navigate", "go", "locate", "enter"], [], [], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#question
	{"params": ["Action_talk", "Question"],
	"Action_talk": [["speak", "answer", "tell", "say"], [], [], []],
	"Question": [[], [], ["question"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type wait_for_user_instruction) (params question -Question-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#find person 1 parametro
	{"params": ["Action_find","Find_person"],
	"Action_find": [["find", "look_for", "locate", "meet"], [], [], []],
	"Find_person": [[], [], ["person"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Find_person-) (step ))" +
	                        "(task (plan user_speech) (action_type find_person_in_room) (params -Find_person-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#find person 2 parametros
	{"params": ["Find_person", "Person", "Person_location"],
	"Find_person": [["find", "locate", "look_for", "spot", "pinpoint"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Person_location": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Person_location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	# deliver to me
	{"params": ["Action_deliver", "Person"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"Person": [["me"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	# deliver to person
	{"params": ["Action_deliver", "Person"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"It": [["it"], [], [], []],
	"Person": [[], [], ["person"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
                                "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	# deliver to person 2 parametros
	{"params": ["Action_deliver", "It", "Person", "Location"],
	"Action_deliver": [["give", "bring", "deliver", "hand"], [], [], []],
	"It": [["it"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location": [[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" +
							"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#grasp object 2 parametros
	{"params": ["Action_get", "Get_object", "Source_get"],
	"Action_get": [["get", "grasp", "take", "retrieve", "pick up"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"Source_get": [[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Get_object- -Source_get-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#grasp object 1 parametro
	{"params": ["Action_get", "Get_object"],
	"Action_get": [["get", "grasp", "take", "retrieve","pick_up"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Get_object-) (step ))" +  
                                "(task (plan user_speech) (action_type get_object) (params -Get_object-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#deliver in place
	{"params": ["Action_place", "Destination_place"],
	"Action_place": [["place", "deliver", "put"], [], [], []],
	"Destination_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type deliver_in_position) (params -Destination_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#follow 1 parametro
	{"params": ["Action_follow", "Pron"],
	"Action_follow": [["follow", "after", "behind"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#follow to room
	{"params": ["Action_follow", "Pron", "Location"],
	"Action_follow": [["follow", "after", "behind"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type follow_man) (params man -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#guide to room
	{"params": ["Action_guide", "Pron", "Location"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany", "conduct"], [], [], []],
	"Pron":[["me","us","you","it","him","her","them"],[],[],[]],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#guide to room 2 parameters
	#{"params": ["Action_guide", "Person", "Location"],
	#"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
#	"Person": [[], [], ["person"], []],
#	"Location":[[], [], ["place"], []],
#	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
#	"verbal_confirmation": '',
#	"planner_confirmed": '',
#	"planner_not_confirmed": ''},
	
        #$vbfind the $object 1 parametro
	{"params": ["Action_get", "Get_object"],
	"Action_get": [["find", "look_for", "locate", "spot", "pinpoint"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"conceptual_dependency":"(task (plan_user_speech) (action_type ask_info) (params question follow_place_origin -Get_object-) (step ))" + 
                                "(task (plan user_speech) (action_type get_object) (params -Get_object-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ###############################
        #### MANIPULATION           ###
        ###############################
	
	#$take to the {placement 2}
	{"params": ["Action_take", "Object", "To", "Place"],
	"Action_take": [[ "get" , "grasp" , "take" , "pick up", "bring", "place"], [], [], []],
	"Object": [[], [], ["item"], []],
	"To": [["to"], [], [], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$vbplace the $object on the {placement 2}
	{"params": ["Action_place", "Object", "Place"],
	"Action_place": [["put", "place", "deliver", "set", "leave"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$deliver  = $vbbring me the $object
	{"params": ["Action_take", "Person", "Object"],
	"Action_take": [["bring", "give", "deliver"], [], [], []],
	"Person": [["me"], [], [], []],
	"Object": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$vbdeliver the $object to $someone
	{"params": ["Action_take", "Object", "Person", "Location"],
	"Action_take": [["bring", "give", "deliver"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Person": [[], [], ["person"], []],
	"Location": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step )) " +
				"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$takefrom to the {placement 2}
	{"params": ["Action_take", "Object","Place_first","To", "Place_second"],
	"Action_take": [["get" , "grasp" , "take" , "pick up", "bring"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Place_first": [[], [], ["place"], []],
	"To": [["to"], [], [], []],
	"Place_second": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Place_first-) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Place_second-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$goplace, $vbfind the $object, and ($delivme | $delivat)
	#$goplace, $vbfind the $object, and $place
       
       ################################
       ### FIND PEOPLE              ###
       ################################

	#$talk to $whowhere    $findp in the {room} and $talk
	{"params": ["Action_talk", "Question", "Person", "Location"],
	"Action_talk": [["speak", "answer", "tell", "say"], [], [], []],
	"Question": [[], [], ["question"], []],
	"Person": [[], [], ["person"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" + 
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question -Question-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$goroom, $findp, and $talk
        
        ###############################
	#### FIND OBJECTS           ###
        ###############################

	#$fndobj   = $vbfind the {kobject?} in the {room}
	{"params": ["Action_get", "Object", "Location"],
	"Action_get": [["find", "look_for", "locate", "spot", "pinpoint"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Location":[[], [], ["place", "room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location -Location-) (step ))"+
				"(task (plan user_speech) (action_type find_object_in_room) (params -Object- -Location-) (step ))",
	"verbal_confirmation": '',	
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	#$fndobj   = Tell me how many $object there are on the {placement}
	{"params": ["Action_talk", "Person", "Many", "Object", "Location_second"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Object": [[], [], ["item"], []],
	"Location_second":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_objects) (params -Object- -Location_second- nil) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ###############################
        #### FOLLOW people          ###
        ###############################

	#$follow   = $vbfollow {name 1} from the {beacon 1} to the {room 2}
	#$follow   = meet {name 1} at the {beacon 1} and $vbfollow them $fllwdest
	{"params": ["Action_follow", "Person", "Location_first", "Location_second"],
	"Action_follow": [["follow", "after"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_first-) (step )) " +
				"(task (plan user_speech) (action_type follow_man) (params man -Location_second-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ###############################
        ### GUIDE PEOPLE            ###
        ###############################

	#$gdcmd    = $vbguide {name 1} from the {beacon 1} to the {beacon 2}
	#$gdcmd    = meet {name 1} at the {beacon 1} and $guideto
	{"params": ["Action_guide", "Person", "Location_first", "Location_second"],
	"Action_guide": [["guide" , "escort" , "take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_first-) (step )) " +
				"(task (plan user_speech) (action_type get_object) (params man_guide -Location_second-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#$gdcmd    = $vbguide {name 1} to the {beacon 2}, $gdwhere
	{"params": ["Action_guide", "Person", "Location_first", "May", "Action_find", "Location_second"],
	"Action_guide": [["guide" , "escort" , "take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place", "room"], []],
	"May":[["may", "can", "will"],[],[],[]],
	"Action_find":[["find"],[],[],[]],
	"Location_second":[[], [], ["place", "room"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_second-) (step )) " +
				"(task (plan user_speech) (action_type get_object) (params man_guide -Location_first-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ###############################
        #### Category II - People   ###
        ###############################
        
        #tell me how many $people there are in the $room
        #tell me how many $peopleR in the $room
        #tell me how many $ppl in the $room are $peopleDsc
        #tell me how many $peoplege in the $room
        {"params": ["Action_talk", "Person", "Many", "PeopleDsc","Ppl", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
        "PeopleDsc":[[],[],["gesture", "colord", "outfit", "posprs", "color"],[]],
        "Ppl":[[],[],["people", "ppl"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency": "(task (plan user_speech) (action_type find_how_many_people) (params -Ppl- -PeopleDsc- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #offer something to (eat | drink) to all the $people in the $room
        #offer something to (eat | drink) to all the $peopleg in the $room
        {"params": ["Action_offer", "EatDrink", "PeopleDsc","Ppl", "Location"],
        "Action_offer":[["offer"],[],[],[]],
        "EatDrink":[["eat", "drink"],[],[],[]],
        "PeopleDsc":[[],[],["gesture", "colord", "outfit", "posprs", "color"],[]],
        "Ppl":[[],[],["people", "ppl"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type offer_eat_drink) (params -Ppl- -PeopleDsc- -EatDrink- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #bring the {objetc} to the $person in the $room
        {"params": ["Action_bring", "Object", "To", "Person", "PeopleDsc", "Location"],
        "Action_bring": [["bring"],[],[],[]],
        "Object":[[],[],["item"],[]],
        "To":[["to"],[],[],[]],
        "Person":[[],[],["gprsn", "people"],[]],
        "PeopleDsc":[[],[],["gesture", "apparel", "property"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type bring_obj_to_prsn) (params -Person- -PeopleDsc- -Object- -Location-) (step ))", 
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed":''},

        #bring the {object} to the $person in the $room
        {"params": ["Action_bring", "Object", "To", "Person", "Color", "Outfit", "Location"],
        "Action_bring": [["bring"],[],[],[]],
        "Object":[[],[],["item"],[]],
        "To":[["to"],[],[],[]],
        "Person":[[],[],["gprsn", "people"],[]],
        "Color":[[],[],["color"],[]],
        "Outfit":[[],[],["outfit"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type bring_obj_to_prsn) (params -Person- -Color- -Outfit- -Object- -Location-) (step ))", 
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed":''},

        #$greet the $person in the $room
        {"params":["Action_greet", "Person", "PeopleDsc", "Location"],
        "Action_greet":[["hello", "introduce_yourself", "greet", "salute", "handshake"],[],[],[]],
        "Person":[[],[],["gprsn", "people"],[]],
        "PeopleDsc":[[],[],["gesture", "property", "apparel"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type greet_person) (params -Person- -PeopleDsc- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #$greet the $person in the $room
        {"params":["Action_greet", "Person", "Color", "Outfit", "Location"],
        "Action_greet":[["hello", "introduce_yourself", "greet", "salute", "handshake"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "Color":[[],[],["color"],[]],
        "Outfit": [[],[],["outfit"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type greet_person) (params -Person- -Color -Outfit- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #$vbfollow the $fgwhor
        #$vbfollow the $fgwho $fbriefing
        {"params":["Action_follow", "Person", "PeopleDsc", "Location"],
        "Action_follow":[["follow", "after", "behind", "accompany"],[],[],[]],
        "Person":[[],[],["gprsn", "people"],[]],
        "PeopleDsc":[[],[],["gesture", "property", "apparel"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type follow_person) (params -Person- -PeopleDsc- -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #$vbfollow the $fgwhor
        {"params":["Action_follow", "Person", "Color", "Outfit", "Location"],
        "Action_follow":[["follow", "after", "behind", "accompany"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "Color":[[],[],["color"],[]],
        "Outfit":[[],[],["outfit"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type follow_person) (params -Person- -Color- -Outfit- -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},
        
        #$vbguide the $fgwhor to the (exit | {room 2})
        {"params":["Action_guide", "Person", "PeopleDsc", "Location_first", "To", "Location_second"],
        "Action_guide":[["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "PeopleDsc":[[],[],["gesture", "property", "apparel"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "To":[["to"],[],[],[]],
        "Location_second":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type guide_person) (params -Person- -PeopleDsc- -Location_first- -Location_second-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #vbguide the $fgwhor to the (exit | {room 2})
        {"params":["Action_guide", "Person", "Color", "Outfit", "Location_first", "Location_second"],
        "Action_guide":[["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "Color":[[],[],["color"],[]],
        "Outfit":[[],[],["outfit"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "Location_second":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type guide_person) (params -Person- -Color- -Outfit- -Location_first- -Location_second-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #tell me how the person $posture at the {beacon} looks like
        #describe the person $posture at the {beacon} to me
        #describe to me the person $posture at the {beacon}
        {"params":["Action_describe", "Person_first", "Person_second", "Posture", "Location"],
        "Action_describe":[["describe", "tell"],[],[],[]],
        "Person_first":[["me"],[],[],[]],
        "Person_second":[["person", "man"],[],[],[]],
        "Posture":[[],[],["posprs"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type describe_person) (params -Posture- -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #describe the person $posture at the {beacon} to the person at the {beacon}
        {"params":["Action_describe", "Person", "Posture", "Location_first", "Location_second"],
        "Action_describe":[["describe"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "Posture":[[],[],["posprs"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "Location_second":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type describe_person) (params -Posture- -Location_first- -Location_second-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #$vbmeet {name} at the {beacon 1}
        {"params":["Action_meet", "Person", "Location"],
        "Action_meet":[["meet", "contact", "know", "acquainted"],[],[],[]],
        "Person":[[],[],["person"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" +
                                "(task (plan user_speech) (action_type remind_person) (params -Person- -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #then (greet | find) {pron} in the $room
        {"params":["Action_greet", "Pron", "Location"],
        "Action_greet":[["greet", "find"],[],[],[]],
        "Pron":[["her", "him", "it"],[],[],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type greet_known_person) (params -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},
    
        ###############################
        #####Incomplete commands    ###
        ###############################

        #$greet {name 1 meta: Provide {name 1} description to the robot} in the $room
        {"params":["Action_greet", "Person", "Location"],
        "Action_greet":[["hello", "introduce_yourself", "greet", "salute", "handshake"],[],[],[]],
        "Person":[[],[],["person"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type greet_known_name) (params -Person- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #offer something to (eat | drink) to {name 1}
        {"params": ["Action_offer", "EatDrink", "Person", "Location"],
        "Action_offer":[["offer"],[],[],[]],
        "EatDrink":[["eat", "drink"],[],[],[]],
        "Person":[[],[],["person"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type offer_eat_drink_known_person) (params -Person- -EatDrink- -Location-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},
        
        #$vbfollow the $fgwhorI
        #$vbfollow the $fgwhoI $fbriefing
        {"params":["Action_follow", "Person", "Location"],
        "Action_follow":[["follow", "after", "behind", "accompany"],[],[],[]],
        "Person":[[],[],["person"],[]],
        "Location":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type follow_known_person) (params -Person- -Location-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #$vbguide the $fgwhorI
        {"params":["Action_guide", "Person", "Location_first", "To", "Location_second"],
        "Action_guide":[["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
        "Person":[[],[],["person"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "To":[["to"],[],[],[]],
        "Location_second":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type guide_known_person) (params -Person- -Location_first- -Location_second-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #$greet the $person in the $room
        {"params":["Action_greet", "Person", "PeopleDsc"],
        "Action_greet":[["hello", "introduce_yourself", "greet", "salute", "handshake"],[],[],[]],
        "Person":[[],[],["gprsn", "people"],[]],
        "PeopleDsc":[[],[],["gesture", "property", "apparel"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type greet_person_no_location) (params -Person- -PeopleDsc-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},
        
        #tell me how many $people there are in the $room
        #tell me how many $peopleR in the $room
        #tell me how many $ppl in the $room are $peopleDsc
        #tell me how many $peoplege in the $room
        {"params": ["Action_talk", "Person", "Many", "PeopleDsc","Ppl"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
        "PeopleDsc":[[],[],["gesture", "colord", "outfit", "posprs", "color"],[]],
        "Ppl":[[],[],["people", "ppl"],[]],
        "conceptual_dependency": "(task (plan user_speech) (action_type find_how_many_people_no_location) (params -Ppl- -PeopleDsc-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},

        #offer something to (eat | drink) to all the $people in the $room
        #offer something to (eat | drink) to all the $peopleg in the $room
        {"params": ["Action_offer", "EatDrink", "PeopleDsc","Ppl"],
        "Action_offer":[["offer"],[],[],[]],
        "EatDrink":[["eat", "drink"],[],[],[]],
        "PeopleDsc":[[],[],["gesture", "colord", "outfit", "posprs", "color"],[]],
        "Ppl":[[],[],["people", "ppl"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type offer_eat_drink_no_location) (params -Ppl- -PeopleDsc- -EatDrink-) (step ))",
        "verbal_confirmation": '',
        "planner_confirmed": '',
        "planner_not_confirmed": ''},


        #$vbguide the $fgwhor to the (exit | {room 2})
        {"params":["Action_guide", "Person", "PeopleDsc", "Location_first"],
        "Action_guide":[["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "PeopleDsc":[[],[],["gesture", "property", "apparel"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type guide_person_no_location) (params -Person- -PeopleDsc- -Location_first-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},
        
        #vbguide the $fgwhor to the (exit | {room 2})
        {"params":["Action_guide", "Person", "Color", "Outfit", "Location_first"],
        "Action_guide":[["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
        "Person":[[],[],["gprsn"],[]],
        "Color":[[],[],["color"],[]],
        "Outfit":[[],[],["outfit"],[]],
        "Location_first":[[],[],["place", "room"],[]],
        "conceptual_dependency":"(task (plan user_speech) (action_type guide_person_no_location) (params -Person- -Color- -Outfit- -Location_first-) (step ))",
        "verbal_confirmation":'',
        "planner_confirmed":'',
        "planner_not_confirmed":''},

        #########
        #### NEW COMMANDS FOR SIDNEY 
        ######
       
        ###
        ## Bring from description
        ###

        #$bringdesc = $vbdeliver me the $bdwhat from the {placement 1}
	{"params": ["Action_deliver", "Person", "Desc","Object", "Place"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person": [["me"], [], [], []],
        "Desc": [[],[],["color", "adjectivea", "abspos", "property"],[]],
	"Object": [["object"], [], [], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type get_object) (params object -Place- -Desc- nil) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$bringdesc = $vbdeliver me the $bdwhat from the {placement 1}
	{"params": ["Action_deliver", "Person", "Desc", "Category", "Place"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person": [["me"], [], [], []],
        "Desc": [[],[],["color", "adjectivea", "abspos", "property"],[]],
	"Category": [[], [], ["category"], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params object -Place- -Desc- -Category-) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$bringdesc = take the $bdwhat from the {placement 1} to the {placement 2}
	{"params": ["Action_take", "Desc", "Object", "Origin_place", "Destiny_place"],
	"Action_take": [["take"], [], [], []],
        "Desc": [[],[],["color", "adjectivea", "abspos", "property"],[]],
	"Object": [["object"], [], [], []],
	"Origin_place": [[], [], ["place"], []],
	"Destiny_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params object -Origin_place- -Desc- nil) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Destiny_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        #$bringdesc = take the $bdwhat from the {placement 1} to the {placement 2}
        {"params": ["Action_take", "Desc", "Category", "Origin_place", "Destiny_place"],
	"Action_take": [["take"], [], [], []],
        "Desc": [[],[],["color", "adjectivea", "abspos", "property"],[]],
	"Category": [[], [], ["category"], []],
	"Origin_place": [[], [], ["place"], []],
	"Destiny_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params object -Origin_place- -Desc- -Category-) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params -Destiny_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$bringdesc = $vbdeliver me the {category} at the $rpos {placement 1}
	{"params": ["Action_deliver", "Person",  "Category", "Rpos","Place"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person": [["me"], [], [], []],
        "Rpos": [["left", "right", "center", "middle_bottom", "top"],[],[],[]],
	"Category": [[], [], ["category"], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type get_rpose_object) (params object -Place- -Rpos- -Category-) (step )) " +
	                        "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params object) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$bringdesc = $vbdeliver me something in a $canpourin
	{"params": ["Action_deliver", "Person",  "Something", "Canpourin"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person": [["me"], [], [], []],
	"Something": [["something"], [], [], []],
	"Canpourin": [[], [], ["item"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type pourin_object) (param nil -Canpourin- -Person-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$bringdesc = take the {category} at the {placement 1}'s $rpos to the {placement 2}
        {"params": ["Action_take", "Category", "Origin_place", "Rpos","Destiny_place"],
	"Action_take": [["take"], [], [], []],
	"Category": [[], [], ["category"], []],
	"Origin_place": [[], [], ["place"], []],
        "Rpos": [["left", "right", "center", "middle_bottom", "top"],[],[],[]],
	"Destiny_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_rpose_object) (params object -Origin_place- -Rpos- -Category-) (step )) " +
				"(task (plan user_speech) (action_type deliver_in_position) (params object -Destiny_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ####
        ## Bring from/to storage
        ###
        
        #$store = put the {object} into the $storage
        {"params": ["Action_put", "Object", "Into", "Storage"],
	"Action_put": [["put"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Into": [["into"], [], [], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type storage_object) (params -Object- default_location -Storage- nil) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        {"params": ["Action_put", "Object", "Into", "Special_object","Storage"],
	"Action_put": [["put"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Into": [["into"], [], [], []],
	"Special_object": [[], [], ["item"], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type storage_object) (params -Object- default_location -Storage- -Special_object-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$store = pick up a {category} from the {placement} and put it into the $storage
        #{"params": ["Action_take", "Category", "Place"],
	#"Action_take": [["take"], [], [], []],
	#"Category": [[], [], ["category"], []],
	#"Place": [[], [], ["place"], []],
	#"conceptual_dependency": "(task (plan user_speech) (action_type storage_object) (params -Category- -Place-) (step )) ",
	#"verbal_confirmation": '',
	#"planner_confirmed": '',
	#"planner_not_confirmed": ''},
        
        {"params": ["Action_put", "It", "Into", "Storage"],
	"Action_put": [["put"], [], [], []],
	"It": [["it"], [], [], []],
	"Into": [["into"], [], [], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type storage_object) (params -Storage- nil) (step ))",
	#"conceptual_dependency":"(task (plan user_speech) (action_type deliver_in_position) (params -Storage-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        {"params": ["Action_put", "It", "Into", "Special_object", "Storage"],
	"Action_put": [["put"], [], [], []],
	"It": [["it"], [], [], []],
	"Into": [["into"], [], [], []],
	"Special_object": [[], [], ["item"], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type storage_object) (params -Storage- -Special_object-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$findordI = $vbgor to the {room}, $vbfind the {object?}
        {"params": ["Action_find", "Category"],
	"Action_find": [["find", "look_for", "locate", "spot", "pinpoint"], [], [], []],
	"Category": [[], [], ["category"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
                                "(task (plan user_speech) (action_type get_object) (params default_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #No se implemento $findorsI = $vbfind (a | the) {object meta: Robot must place it on the {placement}}
        #$takeI = $vbtake the {aobject? meta: Place to } from the {placement}
        {"params": ["Action_take", "Category", "Origin_place"],
	"Action_take": [["take", "get", "retrieve", "grasp", "pick up"], [], [], []],
	"Category": [[], [], ["category"], []],
	"Origin_place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
				"(task (plan user_speech) (action_type get_object) (params default_location -Origin_place- ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        #$takeI = $vbtake the {aobject meta: Robot will find it on the {placement}}
        
        #$place    = $vbplace it on the {placement}
        #$placeI   = $vbplace it to the {placement?}
        {"params": ["Action_place", "Place"],
	"Action_place": [["put", "place", "leave", "set"], [], [], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        {"params": ["Action_place", "Room"],
	"Action_place": [["put", "place", "leave", "set"], [], [], []],
	"Room": [[], [], ["room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object_place -Room-) (step ))" +
                                "(task (plan user_speech) (action_type deliver_in_position) (params default_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$mvobjI   = $vbplace the {object} in the {placement?}
        {"params": ["Action_place", "Object", "Room"],
	"Action_place": [["put", "place", "leave", "set"], [], [], []],
	"Object": [[], [], ["item"], []],
	"Room": [[], [], ["room"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object_place -Room-) (step ))" +
                                "(task (plan user_speech) (action_type get_object) (params -Object- default_location) (step ))" +
                                "(task (plan user_speech) (action_type deliver_in_position) (params default_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$mvobjI   = $vbplace a {object? meta: place 3 objects of the same category at {placement 1}} on the {placement 1}
        {"params": ["Action_place", "Category", "Place"],
	"Action_place": [["put", "place", "leave", "set"], [], [], []],
	"Category": [[], [], ["category"], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question object -Category-) (step ))" +
                                "(task (plan user_speech) (action_type get_object) (params default_object default_location) (step ))" +
                                "(task (plan user_speech) (action_type deliver_in_position) (params -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$findpsI  = $vbfind {name meta: {pron sub} is (sitting | standing | lying | waving ) at the {beacon}}
        {"params": ["Action_find", "Person"],
	"Action_find": [["find", "locate", "spot", "pinpoint", "look_for"], [], [], []],
	"Person": [[], [], ["person"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
	                        "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$deliverI  = $vbdeliver it to {name meta: {pron sub} is (sitting | standing | lying | waving ) at the {beacon}}
        {"params": ["Action_deliver", "Person"],
	"Action_deliver": [["bring", "deliver", "give", "hand"], [], [], []],
	"Person": [[], [], ["person"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question follow_place_origin -Person-) (step ))" +
	                        "(task (plan user_speech) (action_type find_person_in_room) (params -Person-) (step ))", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #$singleSpI= (speak | (say something)) {void meta: When asked, reply to the robot: "$whattosay" }
        {"params": ["Action_say"],
	"Action_say": [["speak", "say"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question whattosay nil) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question default question) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	
        #and $vbguide {pron}	
	{"params": ["Action_guide", "Pron"],
	 "Action_guide": [["guide", "escort", "take", "lead", "accompany", "conduct"],[],[],[]],
	 "Pron": [["her", "him", "it"],[],[],[]],
	 "conceptual_dependency":"(task (plan user_speech) (action_type ask_info) (params question place_destiny person) (step ))" +
	                        "(task (plan user_speech) (action_type get_object) (params man_guide) (step ))",
	 "verbal_confirmation": '',
	 "planner_confirmed": '',
	 "planner_not_confirmed": ''}, 

        #$countobj  = tell $repwho how many $countwhat are in the {placement 1}
	{"params": ["Action_talk", "Person", "Many", "Category", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Category": [["snacks", "cutlery", "food", "drinks", "tableware", "containers", "fruits", "cleaning_stuff"], [], [], []],
	"Location":[[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_category) (params -Category- -Location- nil) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	{"params": ["Action_talk", "Person", "Many", "Color", "Object", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Color":[["red", "blue", "yellow", "black", "white", "gray", "orange"],[],[],[]],
	"Object": [[], [], ["item"], []],
	"Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_objects) (params -Object- -Location- -Color-) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        {"params": ["Action_talk", "Person", "Many", "Color", "Category", "Location"],
	"Action_talk": [["tell"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Many":[["many"],[],[],[]],
	"Color":[["red", "blue", "yellow", "black", "white", "gray", "orange"],[],[],[]],
	"Category": [[], [], ["category"], []],
	"Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_category) (params -Category- -Location- -Color-) (step ))" +
				"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$countobj  = count the $countwhat at the {placement 1} and $report
	{"params": ["Action_count", "Category", "Location"],
	"Action_count": [["count"], [], [], []],
	"Category": [["snacks", "cutlery", "food", "drinks", "tableware", "containers", "fruits", "cleaning_stuff"], [], [], []],
	"Location":[[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_category) (params -Category- -Location- nil) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	{"params": ["Action_count", "Color", "Object", "Location"],
	"Action_count": [["count"], [], [], []],
	"Color":[["red", "blue", "yellow", "black", "white", "gray", "orange"],[],[],[]],
	"Object": [[], [], ["item"], []],
	"Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_objects) (params -Object- -Location- -Color-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        {"params": ["Action_count", "Color", "Category", "Location"],
	"Action_count": [["count"], [], [], []],
	"Color":[["red", "blue", "yellow", "black", "white", "gray", "orange"],[],[],[]],
	"Category": [[], [], ["category"], []],
	"Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_how_many_category) (params -Category- -Location- -Color-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        {"params": ["Action_report", "Person"],
	"Action_report": [["inform", "report"], [], [], []],
	"Person":[["me"],[],[],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_many_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$descobj  = describe the objects in the {placement} to $repwho
        {"params": ["Action_describe", "Object", "Place", "Person"],
	"Action_describe": [["describe"], [], [], []],
	"Object": [["objects", "object"], [], [], []],
	"Place": [[], [], ["place"], []],
	"Person":[["me"],[],[],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type get_object_description) (params object -Place-) (step ))" +
                                "(task (plan user_speech) (action update_object_location) (params location current_loc)(step ))" +
				"(task (plan user_speech) (action_type wait_for_user_instruction) (params question tell_desc_obj) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$retrieve  = $vbdeliver me some {category} from the $storage
        {"params": ["Action_deliver", "Person", "Category", "Storage"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Category": [[], [], ["category"], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type retrieve_object) (params  -Category- -Storage- nil) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        {"params": ["Action_deliver", "Person", "Category", "Special_object","Storage"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Category": [[], [], ["category"], []],
	"Special_object": [[], [], ["item"], []],
	"Storage": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type retrieve_object) (params  -Category- -Storage- -Special_object-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$task      = open the (entrance | exit | corridor) door
        #$task      = close the (entrance | exit | corridor) door
        {"params": ["Action", "Pos", "Door"],
	"Action": [["open", "close"], [], [], []],
	"Pos":[["entrance", "exit", "corridor"],[],[],[]],
	"Door": [["door"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type interact_with_door) (params door -Pos- -Action-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$pour      = $vbdeliver me some $pourable in a $canpourin
        {"params": ["Action_deliver", "Person", "Pourable", "Canpourin"],
	"Action_deliver": [["deliver", "bring", "give"], [], [], []],
	"Person":[["me"],[],[],[]],
	"Pourable": [[], [], ["item"], []],
	"Canpourin": [[], [], ["item"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type pourin_object) (param -Pourable- -Canpourin- -Person-) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$pour      = $vbpour some $pourable in a $canpourin
        {"params": ["Action_pour", "Pourable", "Canpourin"],
	"Action_pour": [["pour", "serve"], [], [], []],
	"Pourable": [[], [], ["item"], []],
	"Canpourin": [[], [], ["item"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type pourin_object) (param -Pourable- -Canpourin- nil) (step )) ",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #$serve     = $vbplace a $tableware on the {placement} and a $cutlery $servewhere
        {"params": ["Action_place", "Tableware", "Place"],
	"Action_place": [["place", "set", "put", "leave"], [], [], []],
	"Tableware": [[], [], ["item"], []],
	"Place": [[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type set_tableware) (params -Tableware- -Place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        {"params": ["Cutlery", "On", "Pos"],
	"Cutlery": [[], [], ["item"], []],
	"On": [["on"], [], [], []],
	"Pos": [["it", "right", "left"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type set_cutlery) (params -Cutlery- -Pos-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	####
	### OFFER objects
	####
	{"params": ["Person_name"],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	{"params": ["My", "Person_name"],
	"My": [["My", "name"], [], [], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#I am
	{"params": ["I", "Person_name"],
	"I": [["i"], [], ["person"], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #I want drink
	{"params": ["Person", "Want", "Favorite_drink"],
	"Person": [["i"], [], [], []],
	"Want": [["want"], [], [], []],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "-Favorite_drink-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
	####
	### GET LOCATION NAMES
	###
	{"params": ["At", "Place"],
	"At": [["at", "in"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "-Place-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	{"params": ["Question"],
	"Question": [[], [], ["question"], []],
	"conceptual_dependency": "-Question-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        {"params": ["Question", "You" , "Dream", "Sheep"],
	"Question": [[], [], ["question"], []],
	"You": [["you"], [], [], []],
	"Dream": [["dream"], [], [], []],
	"Sheep": [["sheep"], [], [], []],
	"conceptual_dependency": "-Question-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}

        ]

meaning_mapping_patterns_restaurant = [
	# patrones para el restaurant 2018 montreal
	############################################# GetNDeliver

	# param: [["palabras", "clave"], ["noun", "vrb", "prep_phrase"], ["categoria", "item", "place", "person"], []]
		# take from and deliver to person
	#{"params": ["Action_get", "Get_object", "Source_get", "Action_deliver", "Destination_person", "Destination_location"],
	#"Action_get": [["get", "grasp", "take"], ["vrb"], [], []],
	#"Get_object": [[], ["noun"], ["item"], []],
	#"Source_get": [[], ["noun"], ["place"], []],
	#"Action_deliver": [["bring", "carry", "deliver", "take"], ["vrb"], [], []],
	#"Destination_person": [[], ["noun", "prep_phrase"], ["person"], []],
	#"Destination_location": [[], ["noun"], ["place"], []],
	#"conceptual_dependency": "(task (plan user_speech) (action_type update_object_location) (params -Get_object- -Source_get- ) (step 1)) " +
	#						"(task (plan user_speech) (action_type get_object) (params -Get_object- -Source_get-) (step 2)) " + 
	#						"(task (plan user_speech) (action_type find_person_in_room) (params -Destination_person- -Destination_location-) (step 3))" + 
	#						"(task (plan user_speech) (action_type handover_object) (params -Get_object-) (step 4))",
	#"verbal_confirmation": '',
	#"planner_confirmed": '',
	#"planner_not_confirmed": ''},

	#Beverage
        #a drink
        #I want a drink
        #drink
	{"params": ["Action_order", "Object_find"],
	"Action_order": [["a", "want"], [], [], []],
	"Object_find": [["beer", "chocolate_milk", "coke", "juice", "lemonade", "tea_bag", "water"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type take_order_beverage) (params drink -Object_find-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	{"params": ["Object_find"],
	"Object_find": [["beer", "chocolate_milk", "coke", "juice", "lemonade", "tea_bag", "water"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type take_order_beverage) (params drink -Object_find-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#Combos
        #food
	{"params": ["Action_order", "Object_find1", "Object_find2"],
	"Action_order": [["a", "want"], [], [], []],
	"Object_find1": [["biscuite", "frosty_fruits", "snakes", "carrot", "cereals", "noodles", "onion", "vegemite", "kiwi", "pear", "cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties", "apple", "orange", "lemon"], [], [], []],
	"Object_find2": [["biscuite", "frosty_fruits", "snakes", "carrot", "cereals", "noodles", "onion", "vegemite", "kiwi", "pear", "cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties", "apple", "orange", "lemon"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type take_order_combo) (params obj1 -Object_find1- obj2 -Object_find2-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	{"params": ["Object_find1", "Object_find2"],
	"Object_find1": [["biscuite", "frosty_fruits", "snakes", "carrot", "cereals", "noodles", "onion", "vegemite", "kiwi", "pear", "cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties", "apple", "orange", "lemon"], [], [], []],
	"Object_find2": [["biscuite", "frosty_fruits", "snakes", "carrot", "cereals", "noodles", "onion", "vegemite", "kiwi", "pear", "cheetos", "doritos", "shapes_chicken", "shapes_pizza", "twisties", "apple", "orange", "lemon"], [], [], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type take_order_combo) (params obj1 -Object_find1- obj2 -Object_find2-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}
        ]

meaning_mapping_patterns_receptionist = [
	# patrones para el receptionist Robocup 2018
	############################################# GetNDeliver

	# param: [["palabras", "clave"], ["noun", "vrb", "prep_phrase"], ["categoria", "item", "place", "person"], []]
		# take from and deliver to person
	#{"params": ["Action_get", "Get_object", "Source_get", "Action_deliver", "Destination_person", "Destination_location"],
	#"Action_get": [["get", "grasp", "take"], ["vrb"], [], []],
	#"Get_object": [[], ["noun"], ["item"], []],
	#"Source_get": [[], ["noun"], ["place"], []],
	#"Action_deliver": [["bring", "carry", "deliver", "take"], ["vrb"], [], []],
	#"Destination_person": [[], ["noun", "prep_phrase"], ["person"], []],
	#"Destination_location": [[], ["noun"], ["place"], []],
	#"conceptual_dependency": "(task (plan user_speech) (action_type update_object_location) (params -Get_object- -Source_get- ) (step 1)) " +
	#						"(task (plan user_speech) (action_type get_object) (params -Get_object- -Source_get-) (step 2)) " + 
	#						"(task (plan user_speech) (action_type find_person_in_room) (params -Destination_person- -Destination_location-) (step 3))" + 
	#						"(task (plan user_speech) (action_type handover_object) (params -Get_object-) (step 4))",
	#"verbal_confirmation": '',
	#"planner_confirmed": '',
	#"planner_not_confirmed": ''},

	#Recognize guest name 
        #My name is
	{"params": ["Receptionist_name", "Person_name"],
	"Receptionist_name": [["My", "name", "is"], [], [], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type receptionist_guest_name) (params name -Person_name-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#I am
	{"params": ["Receptionist_name", "Person_name"],
	"Receptionist_name": [["i"], [], ["person"], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type receptionist_guest_name) (params name -Person_name-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        #Only the name
	{"params": ["Person_name"],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type receptionist_guest_name) (params name -Person_name-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        #My favorite drink is
        #is
	{"params": ["Receptionist_drink", "Favorite_drink"],
	"Receptionist_drink": [["My", "favorite", "drink", "is"], [], [], []],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type receptionist_favorite_drink) (params drink -Favorite_drink-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        #Only the drink
	{"params": ["Favorite_drink"],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type receptionist_favorite_drink) (params drink -Favorite_drink-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        ]

meaning_mapping_patterns_servingdrinks = [
	# patrones para serving Drinks Robocup 2019
	############################################# GetNDeliver

	{"params": ["My", "Person_name"],
	"My": [["My", "name"], [], [], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#I am
	{"params": ["I", "Person_name"],
	"I": [["i"], [], ["person"], []],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	#Only the name
	{"params": ["Person_name"],
	"Person_name": [[], [], ["person"], []],
	"conceptual_dependency": "-Person_name-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #I want drink
	{"params": ["Person", "Want", "Favorite_drink"],
	"Person": [["i"], [], [], []],
	"Want": [["want"], [], [], []],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "-Favorite_drink-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #Only drink
	{"params": ["Favorite_drink"],
	"Favorite_drink": [[], [], ["item"], []],
	"conceptual_dependency": "-Favorite_drink-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}
        ]

meaning_mapping_patterns_where_is_this = [
        ##patron where is this SYDNEY 2019
        #Only drink
	{"params": ["Where", "Is", "Place"],
	"Where": [["where"], [], [], []],
	"Is": [["Is"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "-Place-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #Only drink
	{"params": ["Where", "Is", "Object"],
	"Where": [["where"], [], [], []],
	"Is": [["Is"], [], [], []],
	"Object": [[], [], ["item"], []],
	"conceptual_dependency": "-Object-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ####
	### GET LOCATION NAMES
	###
	{"params": ["At", "Place"],
	"At": [["at", "in"], [], [], []],
	"Place": [[], [], ["place", "room"], []],
	"conceptual_dependency": "-Place-",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}
        ]

meaning_mapping_patterns_catering_comfort = [
        ###Patrones para IROS catering granny Annie's comfort

        ## find a person 1 parametro
	{"params": ["Action_find","Find_person"],
	"Action_find": [["find", "look_for", "search_for", "pinpoint", "spot"], [], [], []],
	"Find_person": [[], [], ["person"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_many_rooms) (params -Find_person-) (step ))" +
                                "(task (plan user_speech) (action_type update_location_coords) (params current_person) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#grasp object 1 parametro
	{"params": ["Action_get", "Get_object"],
	"Action_get": [[ "find", "locate", "search_for", "spot", "get", "grasp", "take", "retrieve", "pickup", "look_for", "pinpoint"], [], [], []],
	"Get_object": [[], [], ["item"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object_many_rooms) (params -Get_object-) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        #bring me an object
	# deliver to me
	{"params": ["Action_get", "Person", "Get_object", "Location"],
	"Action_get": [["give", "bring", "deliver", "hand"], [], [], []],
	"Person": [["me"], [], [], []],
        "Get_object":[[],[],["item"],[]],
        "Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type get_object) (params -Get_object- -Location-) (step ))" +
                                "(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ##deliver to me know object
        {"params": ["Action_get", "Pron", "Person"],
	"Action_get": [["give", "bring", "deliver", "hand"], [], [], []],
	"Pron": [["it"], [], [], []],
	"Person": [["me"], [], [], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
				"(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ##deliver in place
	{"params": ["Action_place", "Destination_place"],
	"Action_place": [["place", "leave", "put", "set", "deliver"], [], [], []],
	"Destination_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type deliver_in_position) (params -Destination_place-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ##deliver to person
	{"params": ["Action_give", "Person", "Destination_place"],
	"Action_give": [["give", "bring", "deliver", "hand", "take"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Destination_place": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Destination_place-) (step ))" +
                                "(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##take object to greeted person
	{"params": ["Action_give", "Pron", "Object", "Location"],
	"Action_give": [["give", "bring", "deliver", "hand", "take"], [], [], []],
	"Pron": [["him", "her"], [], [], []],
        "Object":[[],[],["item"],[]],
	"Location": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Location-) (step ))" +
                                "(task (plan user_spech) (action_type update_object_location) (params location current_person) (step ))" +
                                "(task (plan user_speech) (action_type handover_object) (params ) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##take object from loc1 to loc2
	{"params": ["Action_place", "Object", "Location_first", "Location_second"],
	"Action_place": [["give", "bring", "deliver", "hand", "take", "set"], [], [], []],
        "Object":[[],[],["item"],[]],
        "Location_first":[[],[],["place"],[]],
	"Location_second": [[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params -Object- -Location_first-) (step ))" +
                                "(task (plan user_speech) (action_type deliver_in_position) (params -Location_second-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        #follow me
	{"params": ["Action_follow", "Pron"],
	"Action_follow": [["follow", "escort", "accompany", "take", "lead"], [], [], []],
	"Pron":[["me"],[],[],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" + 
                                "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

        ##follow person 
	{"params": ["Action_follow", "Pron"],
	"Action_follow": [["follow", "escort", "accompany", "take", "lead"], [], [], []],
	"Pron":[["us","you","it","him","her","them"],[],[],[]],
	"conceptual_dependency": "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))" + 
                                "(task (plan user_speech) (action_type update_location_coords) (params current_person) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ##follow know person
        {"params": ["Action_follow", "Person", "Location"],
	"Action_follow": [["follow", "escort", "accompany", "take", "lead"], [], [], []],
	"Person":[[],[],["person"],[]],
	"Location":[[],[],["place"],[]],
	"conceptual_dependency":"(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location-) (step ))" + 
                                "(task (plan user_speech) (action_type follow_man) (params man no_location) (step ))" +
                                "(task (plan user_speech) (action_type update_location_coords) (params current_person) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
        
        ##guide me
	{"params": ["Action_guide", "Pron", "Location"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Pron":[["me"],[],[],[]],
	"Location":[[], [], ["place"], []],
	"conceptual_dependency":"(task (plan user_speech) (action_type update_object_location) (params location current_loc) (step ))" +
                                "(task (plan user_speech) (action_type set_status) (params man_guide went) (step ))" +
                                "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))" +
                                "(task (plan user_speech) (action_type update_lcation_coords) (params current_loc) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	#guide to room
	{"params": ["Action_guide", "Pron", "Location"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Pron":[["us","you","it","him","her","them"],[],[],[]],
	"Location":[[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))" +
                                "(task (plan user_speech) (action_type update_location_coords) (params current_person) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},

	####guide to room 2 parameters
	{"params": ["Action_guide", "Person", "Location"],
	"Action_guide": [["guide" , "escort" ,"take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location":[[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type get_object) (params man_guide -Location-) (step ))",
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''},
	
        ##$gdcmd    = $vbguide {name 1} from the {beacon 1} to the {beacon 2}
	##$gdcmd    = meet {name 1} at the {beacon 1} and $guideto
	{"params": ["Action_guide", "Person", "Location_first", "Location_second"],
	"Action_guide": [["guide" , "escort" , "take" , "lead" , "accompany"], [], [], []],
	"Person": [[], [], ["person"], []],
	"Location_first":[[], [], ["place"], []],
	"Location_second":[[], [], ["place"], []],
	"conceptual_dependency": "(task (plan user_speech) (action_type find_person_in_room) (params -Person- -Location_first-) (step )) " +
				"(task (plan user_speech) (action_type get_object) (params man_guide -Location_second-) (step )) " + 
                                "(task (plan user_speech) (action_type update_location_coords) (params current_person) (step )) ", 
	"verbal_confirmation": '',
	"planner_confirmed": '',
	"planner_not_confirmed": ''}

        ]

verbose = True
#######################
# match the fragmented grounded sentence to a conceptual dependency 
def generate_dependency(G, sentence_dict):
        global meaning_mapping_patterns, used_patterns
	# recibe un diccionario con campos "constituents", "objects", "types", "words", 
	#la primeras dos son listas de strings y las otras son lista de listas
	print "diccionario recibido: ", sentence_dict

	print "palabras:: ", sentence_dict["words"]
	#print "Entrando a generate_dependency:.... ", sentence_dict
	used_objects = []
	solved_dependency = ''
	solved = False
	

	#print "7::       ------------------------------------" if verbose else "",
	print "matching an interpretation" 

	# list of interpretations of each meaning pattern
	# 
	#print "WTF... ", len(meaning_mapping_patterns)
	interpretations_list = []
	id_pattern = 0
	for each_pattern in meaning_mapping_patterns:
		# init template interpretation
		id_pattern += 1
		matched_elements = [[each, ""] for each in each_pattern["params"]]
		#print "hey! elementos a machear: ", matched_elements
		current_interpretation = {"id_pattern": id_pattern, "rank":0.0, "matched_elements":matched_elements, "conceptual_dependency": each_pattern["conceptual_dependency"], "verbal_confirmation": each_pattern["verbal_confirmation"], "planner_confirmed": each_pattern["planner_confirmed"], "planner_not_confirmed": each_pattern["planner_not_confirmed"]}
		used_params = []
		used_objs = []

		for each_param in each_pattern["params"]:
			params_matched = []
			
			# try fetch an element from sentence metadata to match a parameter
			#print "chaaaaa: ", sentence_dict["words"],sentence_dict["objects"], len(sentence_dict["objects"]) -1
			for object_index in range(0, len(sentence_dict["words"])) :
				current_words = sentence_dict["words"][object_index]
				current_constituent = sentence_dict["constituents"][object_index]
				current_types = sentence_dict["types"][object_index]
				current_object = sentence_dict["objects"][object_index]
				#print "trying to match ", current_object, " with ", each_param
				#print "Tamanio de la lista  ", sentence_dict["objects"][object_index]
				# verifying for each_param
				if each_param not in used_params and current_object not in used_objs:
					#check words
					veri_words = len(intersection(current_words, each_pattern[each_param][0])) > 0 or each_pattern[each_param][0] == []
					#print 'palabras clave: ', veri_words
					#check constituyente
					veri_const = current_constituent in each_pattern[each_param][1] or each_pattern[each_param][1] == []
					#print 'contituyente: ', veri_const
					#check semantic type
					veri_type = len(intersection(current_types, each_pattern[each_param][2])) > 0 or each_pattern[each_param][2] == []
					#print 'tipos semanticos: ', veri_type
					
					if veri_type and veri_const and veri_words:
						for each_element in current_interpretation["matched_elements"]:
							if each_element[0] == each_param and each_element[1] == "":
								each_element[1] = current_object
								used_params.append(each_param)
								used_objs.append(current_object)
								#print "----------USED PARAMS: ", used_params
								current_interpretation["rank"] +=  1.0/len(current_interpretation["matched_elements"])

		# rank con base a los numero de elementos en el enunciado respecto al numero de parametros requeridos

		#print "rank: ", current_interpretation["rank"]
		current_interpretation["rank"] = current_interpretation["rank"] * (1.0 - (float(abs(len(sentence_dict["objects"])- len(each_pattern["params"]))))/(len(each_pattern["params"])+len(sentence_dict["objects"])))
		#print "rank: ", current_interpretation["rank"], len(sentence_dict["objects"]),len(each_pattern["params"]),  (float(abs(len(sentence_dict["objects"])- len(each_pattern["params"]))))/(len(each_pattern["params"])+len(sentence_dict["objects"]))
		interpretations_list.append(current_interpretation)

	#print ""
	#print "- LOG: TOTAL OF PATTERNS ANALISED: ", len(interpretations_list)

	ranked_interpretations = sorted(interpretations_list, key=lambda k: k["rank"], reverse=True)

	for each_inter in ranked_interpretations:
		print "matched: " if verbose else "",
		print each_inter["matched_elements"] if verbose else "",
		print "rank: " if verbose else "",
		print each_inter["rank"] if verbose else "",
		print "____" if verbose else ""

	# hasta aqui se tienen las interpretaciones de todos los patrones ordenados por
	# porcentaje de roles tematicos aterrizados

	


	# si existe un patron interpretado completamente genero las expresiones
	# de lo contrario empiezo la interaccion para encontrar el resto 
	#print "_____________ RANKED LIST: ", ranked_interpretations
	if ranked_interpretations[0]["rank"] > 0.2:
		used_patterns[ranked_interpretations[0]["id_pattern"]-1] += 1
		#print "substitude grounded parameters" if verbose else "",
		# sustitucion de simbolos aterrizados en las expresiones
		output_expression = ranked_interpretations[0]["conceptual_dependency"]
		verbal_confirmation = ranked_interpretations[0]["verbal_confirmation"]
		planner_confirmed = ranked_interpretations[0]["planner_confirmed"]
		planner_not_confirmed = ranked_interpretations[0]["planner_not_confirmed"]

		for each_param in ranked_interpretations[0]["matched_elements"]:
			#print "labeL: ", each_param[0]
			#print "value: ", each_param[1]

			output_expression = re.sub('-'+each_param[0]+'-', each_param[1], output_expression)
			
			verbal_confirmation = re.sub('-'+each_param[0]+'-', each_param[1], verbal_confirmation)
			planner_confirmed = re.sub('-'+each_param[0]+'-', each_param[1], planner_confirmed)
			planner_not_confirmed = re.sub('-'+each_param[0]+'-', each_param[1], planner_not_confirmed)

			# falta evaluar expresiones en python


		#print ""
		print "Generated expresion: " + output_expression
		return output_expression

		#print "mensaje de confirmacion: ", verbal_confirmation

		#print "accion al planeador de ser confirmado: ", planner_confirmed

		#print "accion al planeador de no ser confirmado: ", planner_not_confirmed

	else:
		print "The sentence was not fully interpreted"
		return False






# Break multiple commands in the same phrase
def break_sentence(sentence_string):
	commands = []
	words = sentence_string.split(" ")
	# break when comma
	for each in words:
		if "," in each:
			commands.append(" ".join(words[0:words.index(each)]+[re.sub(',', '', each)]))
			words = words[words.index(each)+1:len(words)]
	
	# break when "and"
	if "and" in words:
		commands.append(" ".join(words[0:words.index("and")]))
		commands.append(" ".join(words[words.index("and")+1:len(words)]))
	return commands






# grounds every np from the sentence
# 1) mapping words to ontology vocabulary
# 2) sintactical analisys
# 3) solve syntactical well formed noun phrases
def sentence_grounder(G, sentence):
	sentence = parsing.ontology_words_mapping(sentence)
	print "keywords substitution: " + sentence 
	
	words, ranked_tags = parsing.pos_tagger(G, sentence)	
	#print "2::       ------------------------------------" if verbose else ""
	#print "part-of-speech tags: " + " ".join(ranked_tags[0]) if verbose else ""
	
	np_interpretation = parsing.constituent_chunker(parsing.grammar_np_simple, words, ranked_tags[0])
	#print "3::       ------------------------------------" if verbose else "",
	#print "noun phrases segmentation" if verbose else ""
	#print "chunked pos: " + " ".join(np_interpretation[0]) if verbose else ""
	#print "chunked words: " + " ".join(np_interpretation[1]) if verbose else ""
	#print "noun phrases: " if verbose else "",
	#print np_interpretation[2] if verbose else ""
	
	# constituent level
	constituent_level = np_interpretation[1][:]

	solved_nps = []
	solved_nps_types = []
	solved = True
	print "np_interpretation: ", np_interpretation[2]
	for each in np_interpretation[2]:
		ontology_type, names = noun_phrase_grounder(G, each[0], each[1]) 
		if names == []:
			solved = False
			print "noun phrase can not be grounded"
		solved_nps.append(names)
		solved_nps_types.append(ontology_type)
		print "ontology_type: ", ontology_type
		print "solved_nps_types: ", solved_nps_types
	
	#print "4::       ------------------------------------" if verbose else ""
	#print "grounded noun phrases: " if verbose else "",
	#print solved_nps if verbose else ""

	if solved: 
		# obtain all combinations using solved noun phrases
		# subtitutes solved noun phrases into the sentence
		words_np_solved = []
		chunked_constituents = []
		i = 0
		for each_word in np_interpretation[1]:
			if re.match('NOUN_PHRASE_[0-9]+', each_word):
				words_np_solved.append(solved_nps[i])
				chunked_constituents.append(np_interpretation[2][0])
				i += 1
			else:
				words_np_solved.append(each_word)
				chunked_constituents.append(each_word)
		#print "substituted solved nps: ", words_np_solved
		packed_words = []
		for each_word in words_np_solved:
			if not isinstance(each_word, list):
				packed_words.append([each_word])
			else:
				packed_words.append(each_word)

		#print "packed in lists:::: ", packed_words
		all_words = parsing.all_combinations(packed_words)

		#print "5::       ------------------------------------" if verbose else ""
		print "All sentences: " if verbose else "",
		print  all_words if verbose else ""
		#print "-----> all combinations POS: ", np_interpretation[0]
		
		# up to here all direct grounded commands are contructed therefore
		# they can be further separated in NPs and PPs for verb pattern matching
		analized_sentences = []
		for each_utterance in all_words:
			pp_interpretation = parsing.pp_chunker(parsing.grammar_pp, each_utterance, np_interpretation[0], [])
			


			#print ":XX:D:D:DD:D::D:D:D:D::D:D:D:D::DD::D:D------------"
			#print "constituent_level: ", pp_interpretation[0]
			#print "chunked words: ", pp_interpretation[1]
			#print "noun phrases: ", pp_interpretation[2]
			#print "prepositional phrases: ", pp_interpretation[3]

			object_level = []
			# solving prepositional phrases
			pp_names = []
			for each_pp in pp_interpretation[3]:
				pp_names.append(prepositional_phrase_grounder(G, each_pp[0], each_pp[1])[0]) 
				if pp_names == []:
					solved = False
					print "prepositional phrase: " + each + " can not be grounded"
				#solved_nps.append(names)
			#print "-----------PPPPPP", pp_names
			if solved: 
				# obtain all combinations using solved prep phrases
				# subtitutes solved prep phrases into the sentence
				i = 0
				for each_word in pp_interpretation[1]:
					if re.match('PREP_PHRASE_[0-9]+', each_word):
						object_level.append(pp_names[i])
						i += 1
					else:
						object_level.append(each_word)

			# semantic classes
			# pointer alias
			pos_tags_pp = pp_interpretation[0]
			words_pp = pp_interpretation[1]

			print "-------- pos_tags_pp:, ", pos_tags_pp
			print "solved_nps_types: ", solved_nps_types

			k=0
			#print "generando lista de tipos:"
			semantic_types = []
			for iterator in range(0,len(object_level)):
				#print "consultando tipo del objeto: ", object_level[iterator]
				if object_level[iterator] in G:
					#print "padre en la ontologia"
					semantic_types.append(kb_services.all_superclasses(G, object_level[iterator]))
				elif pp_interpretation[0][iterator] == 'noun' or pp_interpretation[0][iterator] == 'prep_phrase':
					print "buscando tipos de: ", solved_nps_types[k]
					print "super clases are: ", kb_services.all_superclasses(G,solved_nps_types[k])
					semantic_types.append(kb_services.all_superclasses(G,solved_nps_types[k]))
					k = k + 1
				else:
					#print "aca se pone lista vacia"
					semantic_types.append([])

			#semantic_types = [kb_services.all_superclasses(G, each) for each in object_level]
			
			#print "----->  object_level: ", object_level
			#print "----->  constituent_level: ", pp_interpretation[0] 
			#print "----->  semantic types: ", semantic_types

			## cut
			# packing words into constituents
			i=0
			#print "original words:", words
			pp_chunked = parsing.pp_chunker(parsing.grammar_pp, each_utterance, np_interpretation[0], [])
			chunked_final_words = []
			#print "....", pp_chunked[1]
			for each_word in pp_chunked[1]:
					if re.match('PREP_PHRASE_[0-9]+', each_word):
						chunked_final_words.append(pp_chunked[3][i][0])
						i += 1
					else:
						chunked_final_words.append(each_word)
			#print "------------chunked pp final words", chunked_final_words
			np_chunked = parsing.constituent_chunker(parsing.grammar_np_simple, chunked_final_words, pp_chunked[0])
			chunked_final_words = []
			i=0
			sin = 0
			constituent_level =[] #np_chunked[0][:]
			for each_word in np_chunked[1]:
					if isinstance(each_word, str) and re.match('NOUN_PHRASE_[0-9]+', each_word):
						if len(np_chunked[2][i][0]) > 1:
							chunked_final_words.append(np_chunked[2][i][0][0])
							chunked_final_words.append(np_chunked[2][i][0][1])
							constituent_level.append(np_chunked[0][sin])
							constituent_level.append(np_chunked[0][sin])
							print "-------------- np chunked final words ", sin
							sin += 1
							
							print "-------------- np chunked final words ", np_chunked[2][i][0]
							print "-------------- np chunked final words ", len(np_chunked[2][i][0][1])
							print "-------------- FINALWORDS ", chunked_final_words 
						else:
							chunked_final_words.append(np_chunked[2][i][0])
							constituent_level.append(np_chunked[0][sin])
							sin += 1
						
						i += 1
					else:
						chunked_final_words.append(each_word)
						constituent_level.append(np_chunked[0][sin])
						sin += 1
			
			

			## cut

			print "Resumen::       ------------------------------------" if verbose else ""
			print "Words: ", chunked_final_words 
			print "Sintax: ", constituent_level 
			print "Objects:", object_level 
			print "Object type", semantic_types


			analized_sentences.append({"words":chunked_final_words, "constituents": constituent_level, "objects": object_level, "types":semantic_types})
		

		# return a list of dicionaries that
		return analized_sentences
	else:
		print "SOMETHING WRONG! no object matched with the noun phrase"
		sem_types = []
		return []


# solve noun phrases
def noun_phrase_grounder(G, words, pos):	
	
	ontology_type, grounded_objs = solve_np(G, words, pos)
	# get a quantifier three cases all, one or defined by a integer
	if grounded_objs != []:
		if 'existencial' in pos:
			return ontology_type, [grounded_objs[0]]
		elif 'universal' in pos:
			return ontology_type, grounded_objs
		elif 'number' in pos:
			if int(words[pos.index('number')]) < len(grounded_objs):
				return ontology_type, grounded_objs[0:int(words[pos.index('number')])]
			else:
				return ontology_type, grounded_objs
		elif 'idf_pro' in pos:
			return ontology_type, [grounded_objs[0]]
		else:
			return ontology_type, grounded_objs
	else:
		print 'Las palabras subespecificadas se resuelven a nivel del planeador'

		return ontology_type, [ontology_type]


# solve noun phrases
def prepositional_phrase_grounder(G, words, pos):	
	if len(pos) == 2 and pos[1] == "noun":
		return [words[1]]
	else:
		return []
		

def solve_np(G, words, pos):
	#print "solving np: " + " ".join(words) if verbose else "",
	nouns = []
	adjs = []
	atts = []
	vrbs = []
	objs = []
	nums = []
	for i in range(len(words)):
		if pos[i] == 'noun':
			if  len(nouns) == 0:
				nouns.append(words[i])
			else:
				adjs.append(words[i])
		elif pos[i] == 'adj':
			adjs.append(words[i])
		elif pos[i] == 'att':
			atts.append(words[i])
		elif pos[i] == 'vrb':
			vrbs.append(words[i])
		elif pos[i] == 'number':
			nums.append(words[i])
		elif pos[i] == 'idf_pro':
			nouns.append('stuff')
	print 'nouns: ' , nouns , '   adjs: ' , adjs , '   vrbs: ' , vrbs , '   atts: ' , atts 
	# collect all objects of class
	if len(nouns) > 0:
		obj_candidates = kb_services.all_objects(G, nouns[0])
		print 'candidate objects: ', obj_candidates
		# filter objects that has correct properties
		if len(adjs) > 0:
			for each_obj in obj_candidates:
				if kb_services.verify_satisfability_of_objclss(G, each_obj, adjs):
					objs.append(each_obj)
		# filter objects that are object of action verb
		elif len(vrbs) > 0:
			for each_obj in obj_candidates:
				obj_actions = kb_services.get_attribute(G, each_obj, 'is_object_of_action')
				if "is_object_of_action" in obj_actions and vrbs[0] in obj_actions["is_object_of_action"]:
					objs.append(each_obj)		
		
	return nouns[0], objs




# pack information for interpretation
def semantic_type_of_nps(G, pos, words, nps):
	np_count = 0
	semantic_types = words[:]
	for i in range(0,len(words)):
		if re.match('NOUN_PHRASE_[0-9]+', words[i]):
			#print "lol found", nps[np_count][0]
			semantic_types[i] = kb_services.semantic_type(G, nps[np_count][0])
			np_count += 1
		else:
			semantic_types[i] = kb_services.semantic_type(G, words[i])
	return semantic_types


# generate message from dictionary
def generate_nl_response_from_dict(dictio):
	response = ""
	if dictio == {}:
		response = "not information about that, sorry"
	else:
		for each in dictio:
			print "HEY " + each + "  " + dictio[each] if verbose else "",
			response +=" is " + each + " " + " and ".join(dictio[each])
	return response

def generate_nl_response_from_list(ls):
	response = ""
	if ls == []:
		response = "not any known object, sorry"
	else:
		if len(ls) == 1:
			response = ls.pop() + " is "
		else:
			for i in range(0, len(ls) - 1):
				response += ls.pop() + ", "
			response += "and " + ls.pop() + " are "
	return response


def test_solver(sentence_string):
	G = kb_services.load_semantic_network()
	grounded_commands = sentence_grounder(G, sentence_string)
	print "grounded command: ", grounded_commands
	for each_command in grounded_commands:
		expression = generate_dependency(G, each_command)
		print "generated expression to planner: ", expression



#test_solver("deliver an asadkasdale to the kitchen")

