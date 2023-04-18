---
title: VCS, Git
author: Tamas Nagy
---

# 05. VCS, Git

## Lecture

--- 


### Version control, Git

---

![](https://cdn.freebiesupply.com/logos/thumbs/2x/git-logo.png){:style="width:300px" align=right}



- Track changes in a set of files
- Coordinating work among developers
- Who made what changes and when
- Revert back at any time
- Local and remote repos
- Take snapshots of files by making a *commit*
 
---

#### Install

---

```bash
sudo apt install git
```

---

#### Basic commands

---

```bash
git init          # Initialize local git repo
git add <file>    # Add file/files to staging area
git status        # Check status of working tree and staging area
git commit -m "What I've done"    # Commit changes in index
git push          # Push to remote repository
git pull          # Pull latest changes from remote repo
git branch <new_branch_name>
git checkout <branch_name>
git merge <branch_name>   # Merge the branch into the current branch
git config --global user.name "Istvan Szabo"
git config --global user.email "istvan.szabo@gmail.com"

```

!!! tip
    **Personal token megjegyzése:** `git config --global credential.helper store`


!!! tip
    **Windows és Linux óra probléma megoldása:** `timedatectl set-local-rtc 1 --adjust-system-clock`



---

#### GitHub

---

![](https://miro.medium.com/max/719/1*WaaXnUvhvrswhBJSw4YTuQ.png){:style="width:300px" align=right}


```bash
git remote
git clone <link>   # Copy repo into a new directory

# Add remote to repository:
git remote add origin <link>
git push -u origin master
```


!!! note "Some alternatives to GitHub"
    GitLab, BitBucket, Launchpad, Phabricator 

---

## Markdown

---

- Markup language, easy to read
- Text file &rarr; Formatted document
- Widespread usag, e.g., blogs, forums, documentations, readme files, GitHub 
- [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)

---

## Practice

---

### 0: Create a GitHub repo

---

1. Register to GitHub, then create a token.

    ---

2. Create a private repo on GitHub for the `ros2_course` package.


    !!! tip
        **Store personal token:** `git config --global credential.helper store`


    ---


3. Create the local repo, set up remote, then push the package contents to GitHUb (GitHub will also help after the repo is created):

    ```bash
    cd ~/ros2_ws/src/ros2_course
    git init
    git add .
    git commit -m "Initial commit"
    git branch -M main
    git remote add origin <REPO_GITHUB_ADDRESS>.git
    git push -u origin main
    ```



    ---


4. Add a README.md to the ros2_course package with the following content:

    ```markdown
    # ros2_course

    ## About
    
    Something about the package.

    ## Usage
   
    How to *build* and use the package.
   
        cd ~/ros2_ws
        colcon build --symlink-install
    ```
    
   ---

5. Commit and push changes:

    ```bash
    git add .
    git commit -m "Add README"
    git push
    ```

    !!! tip "VCS in Clion"
        The use of GitHub can also be configured in CLion, so you can manage versions in a graphical interface.

---

!!! tip
    **Windows and Linux clock problem:** `timedatectl set-local-rtc 1 --adjust-system-clock`

---

## Useful links

- [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)




