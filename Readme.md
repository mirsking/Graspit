# Graspit!
Graspit souce code cloned from graspit official [svn](http://sourceforge.net/p/graspit/code/HEAD/tree/).

## SVN to GIT
* Use git svn to clone the source code 

	```
	git svn clone svn://svn.code.sf.net/p/graspit/code --no-metadata --trunk=trunk --branches=branches --tags=tags Graspit
	 ```
* Change svn tag to git tag

	```
	git tag 2.3 tags/2.3
 	git tag 2.2_prerelease_1 tags/2.2_prerelease_1 
  	git tag 2.2_prerelease_2 tags/2.2_prerelease_2 
   	git tag 2.3_prerelease_1 tags/2.3_prerelease_1
    git tag 2.3_prerelease_2 tags/2.3_prerelease_2
	```
* Remove useless branches

	```
	git branch -r -d tags/2.2_prerelease_1
 	git branch -r -d tags/2.2_prerelease_2
  	git branch -r -d tags/2.3_prerelease_2
   	git branch -r -d tags/2.3_prerelease_1
    git branch -r -d tags/2.3
	```
* Add Github remote and push to Github

	```
	git remote add origin  git@github.com:mirsking/Graspit.git
 	git push origin master --tags
	```
* Add BULLET branch

	```
	git check BULLET
	git branch -b BULLET
	git push -u origin BULLET
	```
