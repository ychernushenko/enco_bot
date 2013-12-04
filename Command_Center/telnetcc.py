import sys
import telnetlib
from Tkinter import Tk, Text, Button, Frame, BOTH, N, E, S, W, INSERT, DISABLED
from tkFont import Font
import threading
import Queue
import datetime

HOST = "128.237.243.105"
tn = telnetlib.Telnet(HOST)


class GuiPart:
    def __init__(self, master, queue, endCommand):
        self.queue = queue
        self.prevMsg = ''
        self.app = CommandCenterGUI(master, endCommand)
        
    def processIncoming(self):
        while self.queue.qsize():
            try:
                msg = self.queue.get(0)
                if msg and msg != self.prevMsg and  msg != "\n" :                               
                    if msg == 'No minion found. Going to pause state':
                        self.app.resumeButton['state'] = 'enabled'
                    self.app.addMessage(msg)
                    self.prevMsg = msg
            except Queue.Empty:
                pass

class CommandCenterGUI(Frame):
    def __init__(self, parent, endCommand):
        Frame.__init__(self, parent, background="white")   
        self.parent = parent
        self.endCommand = endCommand
        self.initUI()
    
    def initUI(self):
        self.parent.title("Command Center")
        self.pack(fill=BOTH, expand=1)

        self.quitButton = Button(self, text="Quit", command=self.endCommand)
        self.quitButton.grid(row=0, column=0, sticky=N+S+E+W)

        self.startButton = Button(self, text="Start", command=self.startMission)
        self.startButton.grid(row=0, column=1, sticky=N+S+E+W)

        self.resumeButton = Button(self, text="Resume",
                              command=self.resumeMission)
        self.resumeButton.grid(row=0, column=2, sticky=N+S+E+W)

        areaFont = Font(family="Arial",size=12,weight="bold")
        self.area = Text(self, height=600, background = "grey", font = areaFont)
        self.area.grid(row=2, column=0, rowspan = 10, columnspan = 3, sticky=N+S+E+W)

    def startMission(self):
        self.startButton['state'] = 'disabled'
        tn.write('S')

    def resumeMission(self):
        #self.resumeButton['state'] = 'disabled'
        tn.write('R')

    def addMessage(self, data):
        date = datetime.datetime.now().strftime("%I:%M:%S")
        self.area.insert('1.0', date + " >>  " + data)

class ThreadedClient:
    def __init__(self, master):
        self.master = master
        self.queue = Queue.Queue()
        self.gui = GuiPart(master, self.queue, self.endApplication)

        self.running = 1
        self.thread1 = threading.Thread(target=self.workerThread1)
        self.thread1.start()

        self.periodicCall()

    def periodicCall(self):
        self.gui.processIncoming()
        if not self.running:
            import sys
            sys.exit(1)
        self.master.after(100, self.periodicCall)

    def workerThread1(self):
        while self.running:
            data = tn.read_until('\r\n',1)
            self.queue.put(data)

    def endApplication(self):
        self.master.destroy()
        self.running = 0
    
def main():
    root = Tk()
    root.geometry("720x600+0+0")
    client = ThreadedClient(root)
    root.mainloop()
    
if __name__ == '__main__':
    main() 
