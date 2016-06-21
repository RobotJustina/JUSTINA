import time
from pyrobotics import BB
from pyrobotics.messages import Command, Response
import egprs_interpreter



def process_string(c):
    interpreted_command = egprs_interpreter.interpret_command(c.params)
    return Response.FromCommandObject(c, True, interpreted_command)

fmap = {
    'process_string' : process_string
}

def main():
    BB.Initialize(2100, fmap)
    BB.Start()
    BB.SetReady(True)

    BB.Wait()

if __name__ == "__main__":
    main()