default: run


doc: readme.html
	xdg-open readme.html

compiling_setting_robot_environment.html: compiling_setting_robot_environment.md
	pandoc -f markdown -t html compiling_setting_robot_environment.md -c misc/github-pandoc.css -s -o compiling_setting_robot_environment.html

compiling: compiling_setting_robot_environment.html
	xdg-open compiling_setting_robot_environment.html


readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -Rf readme.html *.pyc __pycache__ *~ compiling_setting_robot_environment.html
	make -C robot clean

rob:
	cd robot
	make -C robot

run: rob
	python3 run_simple.py


viewpos: rob
	python3 viewpos.py
