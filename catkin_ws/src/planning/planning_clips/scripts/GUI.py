# -*- coding: utf-8 -*-
'''
@author: arcra
'''

import Tkinter as tk
import tkFileDialog, tkMessageBox, os
from clipsFunctions import clips, _clipsLock
import clipsFunctions

class clipsGUI(object):
    
    def __init__(self):
        
        _clipsLock.acquire()
        clips.DebugConfig.FactsWatched = True 
        clips.DebugConfig.RulesWatched = True
        clips.DebugConfig.FunctionsWatched = False
        clips.DebugConfig.GlobalsWatched = True
        
        filePath = os.path.dirname(os.path.abspath(__file__))
        
        clips.BatchStar(filePath + os.sep + 'CLIPS' + os.sep + 'utils.clp')
         
        _clipsLock.release()
        
        self.topLevelWindow = tk.Tk()
        self.topLevelWindow.bind_all('<KeyPress-Return>', self.runCLIPS)

        self.watchFactsButton = tk.Button(self.topLevelWindow, width = 20, text = 'Watch Facts', bg = 'green', activebackground = 'green', command = self.toggleFactsWatched)
        self.watchRulesButton = tk.Button(self.topLevelWindow, width = 20, text = 'Watch Rules', bg = 'green', activebackground = 'green', command = self.toggleRulesWatched)
        self.watchFunctionsButton = tk.Button(self.topLevelWindow, width = 20, text = 'Watch Functions', bg = 'red', activebackground = 'red', command = self.toggleFunctionsWatched)
        self.watchGlobalsButton = tk.Button(self.topLevelWindow, width = 20, text = 'Watch Globals', bg = 'green', activebackground = 'green', command = self.toggleGlobalsWatched)
        
        self.watchAllButton = tk.Button(self.topLevelWindow, text = 'WATCH ALL', bg = 'red', activebackground = 'red', command = self.toggleALLWatched)

        self.logLevelVar = tk.StringVar(value = 'ERROR')
        self.logLevelLabel = tk.Label(self.topLevelWindow, text = "Log level:")
        self.logLevelINFO = tk.Radiobutton(self.topLevelWindow, text = 'INFO', value = 'INFO', variable = self.logLevelVar, command = self.setLogLevel)
        self.logLevelWARNING = tk.Radiobutton(self.topLevelWindow, text = 'WARNING', value = 'WARNING', variable = self.logLevelVar, command = self.setLogLevel)
        self.logLevelERROR = tk.Radiobutton(self.topLevelWindow, text = 'ERROR', value = 'ERROR', variable = self.logLevelVar, command = self.setLogLevel)
        
        self.cmdFrame = tk.Frame(self.topLevelWindow)
        
        self.cmdVar = tk.StringVar()
        self.cmdLabel = tk.Label(self.cmdFrame, text = 'Enter command:')
        self.cmdEntry = tk.Entry(self.cmdFrame, width = 56, textvariable = self.cmdVar)
        self.cmdButton = tk.Button(self.topLevelWindow, width = 20, text = "SEND COMMAND", bg = 'blue', activebackground = 'blue', fg = 'white', activeforeground = 'white', command = self.sendCommand)
        
        self.loadFrame = tk.Frame(self.topLevelWindow)
        
        self.fileVar = tk.StringVar()
        self.fileLabel = tk.Label(self.loadFrame, text = 'File:')
        self.fileEntry = tk.Entry(self.loadFrame, width = 66, textvariable = self.fileVar)
        self.fileEntry.bind('<Button-1>', self.getFileName)
        self.loadButton = tk.Button(self.topLevelWindow, width = 20, text = "LOAD FILE", bg = 'blue', activebackground = 'blue', fg = 'white', activeforeground = 'white', command = self.loadFile)
        
        self.printFactsButton = tk.Button(self.topLevelWindow, width = 20, text = 'Print Facts', bg = 'white', activebackground = 'white', command = clipsFunctions.PrintFacts)
        self.printRulesButton = tk.Button(self.topLevelWindow, width = 20, text = 'Print Rules', bg = 'white', activebackground = 'white', command = clipsFunctions.PrintRules)
        self.printAgendaButton = tk.Button(self.topLevelWindow, width = 20, text = 'Print Agenda', bg = 'white', activebackground = 'white', command = clipsFunctions.PrintAgenda)
        self.resetButton = tk.Button(self.topLevelWindow, width = 20, text = 'RESET', bg = 'blue', activebackground = 'blue', fg = 'white', activeforeground = 'white', command = self.reset)
        
        self.timesFrame = tk.Frame(self.topLevelWindow)
        
        self.timesTextVar = tk.StringVar(value = 0)
        self.runTimesLabel = tk.Label(self.timesFrame, text = 'Run # times: (0 to run ALL)')
        self.runTimesEntry = tk.Entry(self.timesFrame, width = 2, textvariable = self.timesTextVar)
        self.runButton = tk.Button(self.topLevelWindow, text = "RUN", bg = 'blue', activebackground = 'blue', fg = 'white', activeforeground = 'white', command = self.runCLIPS)
        
        self.watchFunctionsButton.grid({'row':0, 'column': 0})
        self.watchGlobalsButton.grid({'row':0, 'column': 1})
        self.watchFactsButton.grid({'row':0, 'column': 2})
        self.watchRulesButton.grid({'row':0, 'column': 3})
        self.watchAllButton.grid({'row': 1, 'column': 0, 'columnspan': 4, 'sticky': tk.E+tk.W})
        
        self.logLevelLabel.grid({'row': 2, 'column': 0})
        self.logLevelINFO.grid({'row': 2, 'column': 1})
        self.logLevelWARNING.grid({'row': 2, 'column': 2})
        self.logLevelERROR.grid({'row': 2, 'column': 3})
        
        self.cmdFrame.grid({'row': 3, 'column': 0, 'columnspan': 3, 'sticky': tk.E+tk.W})
        self.cmdLabel.grid({'row': 0, 'column': 0})
        self.cmdEntry.grid({'row': 0, 'column': 1, 'sticky': tk.E+tk.W})
        self.cmdButton.grid({'row': 3, 'column': 3})
        
        self.loadFrame.grid({'row': 4, 'column': 0, 'columnspan': 3, 'sticky': tk.E+tk.W})
        self.fileLabel.grid({'row': 0, 'column': 0})
        self.fileEntry.grid({'row': 0, 'column': 1, 'sticky': tk.E+tk.W})
        self.loadButton.grid({'row': 4, 'column': 3})
        
        self.printFactsButton.grid({'row': 5, 'column': 0})
        self.printRulesButton.grid({'row': 5, 'column': 1})
        self.printAgendaButton.grid({'row': 5, 'column': 2})
        self.resetButton.grid({'row': 5, 'column': 3})
        
        self.timesFrame.grid({'row': 6, 'column': 0, 'columnspan': 3, 'sticky': tk.E})
        self.runTimesLabel.grid({'row': 0, 'column': 0})
        self.runTimesEntry.grid({'row': 0, 'column': 1})
        self.runButton.grid({'row': 6, 'column': 3, 'sticky': tk.N+tk.S+tk.E+tk.W})

    def toggleFactsWatched(self):
        
        _clipsLock.acquire()
        clips.DebugConfig.FactsWatched = not clips.DebugConfig.FactsWatched
        
        if clips.DebugConfig.FactsWatched:
            self.watchFactsButton['bg'] = 'green'
            self.watchFactsButton['activebackground'] = 'green'
        else:
            self.watchFactsButton['bg'] = 'red'
            self.watchFactsButton['activebackground'] = 'red'
        
        count = clips.DebugConfig.FactsWatched + clips.DebugConfig.RulesWatched \
                + clips.DebugConfig.FunctionsWatched + clips.DebugConfig.GlobalsWatched
        
        _clipsLock.release()
        
        if count == 4:
            self.watchAllButton['bg'] = 'green'
            self.watchAllButton['activebackground'] = 'green'
        else:
            self.watchAllButton['bg'] = 'red'
            self.watchAllButton['activebackground'] = 'red'

    def toggleRulesWatched(self):
        
        _clipsLock.acquire()
        clips.DebugConfig.RulesWatched = not clips.DebugConfig.RulesWatched
        
        if clips.DebugConfig.RulesWatched:
            self.watchRulesButton['bg'] = 'green'
            self.watchRulesButton['activebackground'] = 'green'
        else:
            self.watchRulesButton['bg'] = 'red'
            self.watchRulesButton['activebackground'] = 'red'
        
        count = clips.DebugConfig.FactsWatched + clips.DebugConfig.RulesWatched \
                + clips.DebugConfig.FunctionsWatched + clips.DebugConfig.GlobalsWatched
        
        _clipsLock.release()
        
        if count == 4:
            self.watchAllButton['bg'] = 'green'
            self.watchAllButton['activebackground'] = 'green'
        else:
            self.watchAllButton['bg'] = 'red'
            self.watchAllButton['activebackground'] = 'red'

    def toggleFunctionsWatched(self):
        
        _clipsLock.acquire()
        clips.DebugConfig.FunctionsWatched = not clips.DebugConfig.FunctionsWatched
        
        if clips.DebugConfig.FunctionsWatched:
            self.watchFunctionsButton['bg'] = 'green'
            self.watchFunctionsButton['activebackground'] = 'green'
        else:
            self.watchFunctionsButton['bg'] = 'red'
            self.watchFunctionsButton['activebackground'] = 'red'
        
        count = clips.DebugConfig.FactsWatched + clips.DebugConfig.RulesWatched \
                + clips.DebugConfig.FunctionsWatched + clips.DebugConfig.GlobalsWatched
        
        _clipsLock.release()
        
        if count == 4:
            self.watchAllButton['bg'] = 'green'
            self.watchAllButton['activebackground'] = 'green'
        else:
            self.watchAllButton['bg'] = 'red'
            self.watchAllButton['activebackground'] = 'red'
    
    def toggleGlobalsWatched(self):
        
        _clipsLock.acquire()
        clips.DebugConfig.GlobalsWatched = not clips.DebugConfig.GlobalsWatched
        
        if clips.DebugConfig.GlobalsWatched:
            self.watchGlobalsButton['bg'] = 'green'
            self.watchGlobalsButton['activebackground'] = 'green'
        else:
            self.watchGlobalsButton['bg'] = 'red'
            self.watchGlobalsButton['activebackground'] = 'red'
        
        count = clips.DebugConfig.FactsWatched + clips.DebugConfig.RulesWatched \
                + clips.DebugConfig.FunctionsWatched + clips.DebugConfig.GlobalsWatched
        
        _clipsLock.release()
        
        if count == 4:
            self.watchAllButton['bg'] = 'green'
            self.watchAllButton['activebackground'] = 'green'
        else:
            self.watchAllButton['bg'] = 'red'
            self.watchAllButton['activebackground'] = 'red'
    
    def toggleALLWatched(self):
        
        _clipsLock.acquire()
        count = clips.DebugConfig.FactsWatched + clips.DebugConfig.RulesWatched \
                + clips.DebugConfig.FunctionsWatched + clips.DebugConfig.GlobalsWatched
        
        if count < 4:
            if not clips.DebugConfig.FactsWatched:
                clips.DebugConfig.FactsWatched = True
                self.watchFactsButton['bg'] = 'green'
                self.watchFactsButton['activebackground'] = 'green'
            if not clips.DebugConfig.RulesWatched:
                clips.DebugConfig.RulesWatched = True
                self.watchRulesButton['bg'] = 'green'
                self.watchRulesButton['activebackground'] = 'green'
            if not clips.DebugConfig.FunctionsWatched:
                clips.DebugConfig.FunctionsWatched = True
                self.watchFunctionsButton['bg'] = 'green'
                self.watchFunctionsButton['activebackground'] = 'green'
            if not clips.DebugConfig.GlobalsWatched:
                clips.DebugConfig.GlobalsWatched = True
                self.watchGlobalsButton['bg'] = 'green'
                self.watchGlobalsButton['activebackground'] = 'green'
            self.watchAllButton['bg'] = 'green'
            self.watchAllButton['activebackground'] = 'green'
        else:
            if clips.DebugConfig.FactsWatched:
                clips.DebugConfig.FactsWatched = False
                self.watchFactsButton['bg'] = 'red'
                self.watchFactsButton['activebackground'] = 'red'
            if clips.DebugConfig.RulesWatched:
                clips.DebugConfig.RulesWatched = False
                self.watchRulesButton['bg'] = 'red'
                self.watchRulesButton['activebackground'] = 'red'
            if clips.DebugConfig.FunctionsWatched:
                clips.DebugConfig.FunctionsWatched = False
                self.watchFunctionsButton['bg'] = 'red'
                self.watchFunctionsButton['activebackground'] = 'red'
            if clips.DebugConfig.GlobalsWatched:
                clips.DebugConfig.GlobalsWatched = False
                self.watchGlobalsButton['bg'] = 'red'
                self.watchGlobalsButton['activebackground'] = 'red'
            self.watchAllButton['bg'] = 'red'
            self.watchAllButton['activebackground'] = 'red'
        
        _clipsLock.release()
    
    def setLogLevel(self):
        _clipsLock.acquire()
        clips.SendCommand('(bind ?*logLevel* ' + self.logLevelVar.get() + ')')
        clipsFunctions.PrintOutput()
        _clipsLock.release()
    
    def sendCommand(self):
        _clipsLock.acquire()
        clips.SendCommand(self.cmdVar.get(), True)
        clipsFunctions.PrintOutput()
        _clipsLock.release()
    
    def getRunTimes(self):
        try:
            times = int(self.timesTextVar.get())
        except:
            times = 0
        self.timesTextVar.set(str(times))
        if times < 1:
            times = ''
        return times
    
    def reset(self):
        clipsFunctions.Reset()
        print 'Facts were reset!'
        self.setLogLevel()
    
    def runCLIPS(self, *args):
        clipsFunctions.Run(self.getRunTimes())
        clipsFunctions.PrintOutput()
    
    def getFileName(self, *args):
        self.fileVar.set(tkFileDialog.askopenfilename(filetypes = [('All possible files', '.clp'), ('All possible files', '.dat'), ('All possible files', '.lst'), ('CLIPS Batch file', '.clp'), ('File list', '.dat'), ('File list', '.lst')]))

    def putFileName(self, string):
        print 'test put file name'
	self.fileVar.set(string)
        print 'test RETURN PUT file name'
    
    def loadFile(self):
        filePath = self.fileVar.get()
        if not filePath:
            tkMessageBox.showinfo('LOAD FILE', 'Click on the text box to select a file to be loaded.')
            return
        
        if filePath[-3:] == 'clp':
            _clipsLock.acquire()
            clips.BatchStar(filePath)
            clipsFunctions.PrintOutput()
            _clipsLock.release()
            print 'File Loaded!'
            return
        
        path = os.path.dirname(os.path.abspath(filePath))
        f = open(filePath, 'r')
        line = f.readline()
        _clipsLock.acquire()
        while line:
            clips.BatchStar((path + os.sep + line).strip())
            line = f.readline()
        f.close()
        clipsFunctions.PrintOutput()
        _clipsLock.release()
        
        print 'Files Loaded!'

def StartGUI():
    
    clipsGUI()
    
    tk.mainloop()

if __name__ == '__main__':
    StartGUI()
