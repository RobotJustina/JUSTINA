# code dependencies
import kb_services
import parsing
import interpretation
# network toolkit
import networkx as nx
# regular expressions 
import re
# drawing



G = kb_services.load_semantic_network()
#print G.nodes()

interpreted_sentences = 0
correctly_interpreted = []
misinterpreted = []
correctly_interpreted_counter = 0
misinterpreted_counter = 0


file = open('ejemplos_rockin.txt', 'r')
file_report = open('resultdos_rockin', 'w')
lines = file.readlines()
file.close()

print "lines: ", lines

#lines = ["Drop the jar", "Put the can on the counter", "Find the glass in the living room", "Search for the glass in the kitchen", "Go to the dining room", "Move along the wall", "Take the cereal box", "Grab the mayo on the table", "remove the sheets from the bed"]

inicio = 0
fin = len(lines)

for iterator in range(inicio,fin):
	print "*********************************************"
	command = lines[iterator]
	command = re.sub('\n', '', command)
	print ""
	print "- OUT: ORIGINAL SENTENCE: ", command
	analized_sentences = interpretation.sentence_grounder(G, command)
	commands = []
	for each_caracterized_sentence in analized_sentences:
		#print ""
		#print "- LOG: GROUNDED SENTENCE: ", each_caracterized_sentence["objects"]
		commands.append(interpretation.generate_dependency(G, each_caracterized_sentence))
		if commands != [False]:
			interpreted_sentences
			interpreted_sentences += 1
			commands[0] = re.sub(' \)', ')', commands[0])
			commands[0] = re.sub('_', ' ', commands[0])
			#print "- LOG: Enunciado generado: ", commands[0]
			file_report.write(command + ' | ' + commands[0] + '\n')
file_report.close()


print "distribucion de interpretaciones: ", interpretation.used_patterns

print parsing.unknown_list
#	print "commands to planner..."	
#	for each in commands:
#		print "sent to planner: ", each
#		print "planner response:"
#		planner_bridge.launch_planner(each)
		
	
# knowledge basesolved_elements[each_element]