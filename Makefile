default: run


doc: readme.html
	xdg-open readme.html


readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -Rf readme.html *.pyc __pycache__ *~
	make -C robot clean

rob:
	cd robot
	make -C robot

run: rob
	python3 run_simple.py


viewpos: rob
	python3 viewpos.py
