default: rob

kill: rob # Kill the robot, reasonably gently
	@if pgrep -x "robot" > /dev/null ; then echo "\n\n-- Killing with Python --\n\n" ;python3 -c "import robot.interface as robot; robot.start_shm(); robot.unload();" ; else echo "\n\n-- No robot process running --\n\n"; fi

doc: readme.html robot.html shm.html
	xdg-open readme.html

compiling_setting_robot_environment.html: compiling_setting_robot_environment.md
	pandoc -f markdown -t html compiling_setting_robot_environment.md -c misc/github-pandoc.css -s -o compiling_setting_robot_environment.html

compiling: compiling_setting_robot_environment.html
	xdg-open compiling_setting_robot_environment.html

dox: doxygen

doxygen:
	make -C robot doxygen

readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -Rf readme.html *.pyc __pycache__ *~ compiling_setting_robot_environment.html robot.html shm.html
	make -C robot clean

rob:
	cd robot
	make -C robot

run: rob
	python example_simple.py


proto: rob
	python3 example_proto.py

viewpos: rob
	python example_viewpos.py


cursor: rob
	python2.7 example_cursor.py

