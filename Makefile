default: rob

kill: rob # Kill the robot, reasonably gently
	python3 -c "import robot; robot.start_shm(); robot.unload();"

doc: readme.html robot.html
	xdg-open readme.html

robot.html:
	pydoc3 -w robot

readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -Rf readme.html robot.html *.pyc __pycache__ *~
	make -C robot clean

rob:
	cd robot
	make -C robot

run: rob
	python3 example_simple.py


proto: rob
	python3 example_proto.py

viewpos: rob
	python3 example_viewpos.py
