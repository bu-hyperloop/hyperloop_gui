QTUI=main_window.ui
QTPY=$(QTUI:.ui=.py)

FILES=main.py
COMPILED=$(FILES:.py=.pyc) $(QTPY:.py=.pyc)

all: 
	pyuic4 $(QTUI) -o $(QTPY)

run: all
	python main.py

clean:
	rm $(COMPILED) $(QTPY)
