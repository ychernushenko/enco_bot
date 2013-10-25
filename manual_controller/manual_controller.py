import socket
import kivy
kivy.require('1.0.6') # replace with your current kivy version !

from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout

from time import sleep

HOST = '128.237.255.22' #'192.168.43.221' #'128.237.127.46' #'10.0.0.11'#
PORT = 50007              

def send_command(command):
    failed = True
    try:
        print command
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send(command + "\n")
        try:
            byte = s.recv(1)
            print "Answer is %s" % byte
            failed = False
        except socket.timeout:
            print "Socket timeout"
            failed = True
    except:
        print "Problem with connection"
    finally:
        s.close()
    return failed

class RoboScreen(GridLayout):
    def __init__(self, **kwargs):
        super(RoboScreen, self).__init__(**kwargs)
        self.cols = 3
        self.rows = 2
        def Q(obj): send_command('q')
        def W(obj): send_command('w')
        def E(obj): send_command('e')
        def S(obj): send_command('s')
        def A(obj): send_command('a')
        def D(obj): send_command('d')
        def H(obj): 
            fail = send_command('h')
            while fail:
                fail = send_command('h')

        btnQ = Button(text='Q')
        btnQ.bind(on_press = Q)
        btnQ.bind(on_release = H)
        self.add_widget(btnQ)

        btnW = Button(text='W')
        btnW.bind(on_press = W)
        btnW.bind(on_release = H)
        self.add_widget(btnW)

        btnE = Button(text='E')
        btnE.bind(on_press = E)
        btnE.bind(on_release = H)
        self.add_widget(btnE)

        btnA = Button(text='A')
        btnA.bind(on_press = A)
        btnA.bind(on_release = H)
        self.add_widget(btnA)

        btnS = Button(text='S')
        btnS.bind(on_press = S)
        btnS.bind(on_release = H)
        self.add_widget(btnS)

        btnD = Button(text='D')
        btnD.bind(on_press = D)
        btnD.bind(on_release = H)
        self.add_widget(btnD)

class RoboApp(App):
    def build(self):
        return RoboScreen()

if __name__ == '__main__':
    RoboApp().run()
