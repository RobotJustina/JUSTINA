import time
import egprs_interpreter
import sys

filename = sys.argv[1]
with open(filename) as f:
	content = f.read()
		
interpreted_command = egprs_interpreter.interpret_command(content)

print " "
print "Comando Interpretado:"
print interpreted_command
