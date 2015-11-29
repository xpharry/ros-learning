1. how to revert to a history state?

	git checkout master
	git reset --hard e3f1e37
	git push --force origin master
	# Then to prove it (it won't print any diff)
	git diff master..origin/master
