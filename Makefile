default: run


doc: readme.html
	xdg-open readme.html


readme.html: readme.md
	pandoc -f markdown -t html readme.md -c misc/github-pandoc.css -s -o readme.html

clean:
	rm -f readme.html




run:
	python3 run_simple.py

