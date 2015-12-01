1. how to revert to a history state?

	git checkout master
	git reset --hard e3f1e37
	git push --force origin master
	# Then to prove it (it won't print any diff)
	git diff master..origin/master

2. how to initialize a local branch?

	1) create a remote branch origin/Emma in github

	2) download a repo from remote:

		git clone https://github.com/ROS-Group-epsilon/MerryClustering.git

	3) switch to the remote branch

		git checkout origin/Emma

	4) create a local branch and switch to that branch

		git branch Emma

		git checkout Emma

	5) set github account name and email

		git config --local user.email "johndoe@company.com"

		git config --local user.name "John Doe"

	6) push your change to remote branch

		git push origin/Emma

	*7) when your code is ready, which means it has no errors and functions well, you can push that to master branch

	   before that you need to sync to the master branch

		git pull

	   put in what you wanna upload and excute:

		git push origin/master
