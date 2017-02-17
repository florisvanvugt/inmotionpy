default: run


doc: readme.html
	xdg-open readme.html


readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -f readme.html
	make -C robot clean
	rm *.pyc

rob:
	cd robot
	make -C robot

run: rob
	python3 run_simple.py

