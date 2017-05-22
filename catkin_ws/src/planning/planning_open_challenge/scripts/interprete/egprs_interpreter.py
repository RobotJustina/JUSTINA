# code dependencies
import kb_services
import parsing
import interpretation

# network toolkit
import networkx as nx
# regular expressions 
import re
# drawing

#from pyrobotics.messages import Command #yo lo quite

def interpret_command(sentence_string):

	print "lol"

	G = kb_services.load_semantic_network()
	grounded_commands = interpretation.sentence_grounder(G, sentence_string)
	print "loll"
	print "grounded command: ", grounded_commands
	for each_command in grounded_commands:
		expression = interpretation.generate_dependency(G, each_command)
		print "generated expression to planner: ", expression
	print "lolll"
#	sentences  = interpretation.break_sentence(sentence_string)
#	print "hi: ", sentences
#	for command in sentences[0:1]:
#		grounded_commands = interpretation.sentence_grounder(G, command)
#		print "grounded command: ", grounded_commands
#		
#		for each_command in grounded_commands:
#			expression = interpretation.generate_dependency(G, each_command)
#			print "output expression: ", expression
#			if commands != [False]:
#				interpreted_sentences
#				interpreted_sentences += 1
#				commands[0] = re.sub(' \)', ')', commands[0])
#				commands[0] = re.sub('_', ' ', commands[0])
				
	return expression

#print " test: ", interpret_command("take the orange juice from the shelf and deliver it to carmen at the dining room")


	



















#print " test: ", interpret_command("take the milk from the kitchen and deliver it to john in the living room")

#print " --------------------------------- "

#print " test: ", interpret_command("take the milk from the kitchen and deliver it to me")

#print " --------------------------------- "

#print " test: ", interpret_command("take the milk from the kitchen and deliver it to the dining room")


