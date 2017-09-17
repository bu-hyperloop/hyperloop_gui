import sys
import os
import threading
from PyQt4 import QtGui, QtCore
import main_window
import time

class MainApp(QtGui.QMainWindow, main_window.Ui_MainWindow):

    """ --- Initialization Step --- """
    # Initialize label values
    valTime = 0.00
    valTapeCount = 0
    valPosition = 0
    valAccX = 0.00

    # Declare threading targets
    runMainLoop = False

    def __init__(self, parent=None):
        super(MainApp, self).__init__(parent)
        self.setupUi(self)

        self.updateLabels() # calls helper method

        self.startButton.clicked.connect(self.startMainLoop) # connect startButton to mainLoop member method
        self.stopButton.clicked.connect(self.stopMainLoop)

    """ --- END initialization step ---"""

    """ Helper Methods """
    def updateLabels(self): #Helper function to update values of labels
        self.labelValTime.setText(QtCore.QString( str(round(self.valTime, 2)) ))
        self.labelValTapeCount.setText(QtCore.QString( str(self.valTapeCount) ))
        self.labelValPosition.setText(QtCore.QString( str(self.valPosition) ))
        self.labelValAccX.setText(QtCore.QString( str(round(self.valAccX, 2)) ))

    def startMainLoop(self): #Helper function to create a new thread for mainLoop
        self.runMainLoop = True
        threading.Thread(target=self.mainLoop).start()

    def stopMainLoop(self): # Helper function to end thread for mainLoop
        setattr(self, "runMainLoop", False)

    def mainLoop(self): # A dummy mainLoop for now, but increments all member values
        while getattr(self, "runMainLoop", True):
            self.valTime+=1
            self.valTapeCount+=1
            self.valPosition+=1
            self.valAccX+=1
            time.sleep(1)  
            self.updateLabels()
    
def main():
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = MainApp()                    # We set the form to be our ExampleApp(design)
    form.show()                         # Render the form
    app.exec_()                         # Execute the app

if __name__ == '__main__': # if we're running the file directly and not importing it
    main()                 # run the main function
