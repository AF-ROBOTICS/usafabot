# Intro and GIT
---


**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/Module0_Intro_GIT.ipynb)



## Syllabus
You can find the syllabus in the repository within the [curriculum folder](../ECE495_Syllabus.pdf). The syllabus may be updated throughout the semester to adapt to changes in the schedule.

## Create GIT Repositories for the **Master** and **Robot** systems.
We will utilize two git repositories to submit code written throughout this course.

1. Browse to [github.com](www.github.com) and create a GitHub account if you do not already have one. It is useful if your username is something that identifies you (e.g., bneff1013).

1. ***One student per group:***
    1. Browse to [https://classroom.github.com/a/woMT5_c1](https://classroom.github.com/a/woMT5_c1).
    1. Select "Accept this assignment"
    1. You may need to hit refresh, but eventually it will provide you a link to the repository.
    1. Browse to your repository.
	1. Note the url for your repository (save this link, it is the best way to check if your repo is updated).
	1. Go to Settings -> Manage access -> and "Invite teams or people".
	1. Provide access to your team member using their GitHub user name.
	1. Browse to [https://classroom.github.com/a/58EyOwaM](https://classroom.github.com/a/58EyOwaM) and repeat steps B-G.

## Enable SSH connections to your GitHub account
1. Open a terminal on your **Master** (`ctrl+alt+t`)
1. The same student as above do the following:

    1. Generate a new SSH key, substituting your GitHub email address:
    ```bash
    ssh-keygen -t ed25519 -C "your_email@example.com"
    ```
    1. When prompted to "Enter a file in which to save the key," click enter.
    1. At the prompt, type a secure passphrase.
    1. Start the ssh-agent and add your SSH private key to the ssh-agent:
    ```bash
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519
    ```
    1. Open the public key:
    ```bash
    nano ~/.ssh/id_ed25519.pub
    ```
    1. Select the contents of the file (maximize the window and ensure it has your GIT email at the end), right click, and select copy.
    1. Open a web browser and sign in to your GitHub account.
    1. In the upper-right corner of any page, click your profile photo, then click **Settings**:
    ![logo](Figures/userbar-account-settings.PNG)
    1. In the user settings sidebar, click **SSH and GPG keys**:
    ![logo](Figures/settings-sidebar-ssh-keys.PNG)
    1. Click **New SSH key**:
    ![logo](Figures/ssh-add-ssh-key.png)
    1. In the "Title" field, add a descriptive label for the new key, such as "MasterX".
    1. Paste your key into the "Key" field (contents of the .pub file).
    1. Click **Add SSH key**.
    1. If prompted, confirm your GitHub password.
    1. Create a SSH connetion to your **Robot** (password is dfec3141):
    ```bash
    ssh pi@robotX
    ```
    1. Repeat steps A-F on your **Robot** and J-N on your **Master**.

## Clone **Master** repository
1. On the **Master**, open the **master** GitHub repository and copy your repo address using the SSH mode:
![logo](Figures/clone.png)
1. Open a terminal and browse to your workspace source folder:
```bash
cd ~/master_ws/src/
```
1. Clone your repo using the username and password used when you generated the SSH key (pasting the copied address above after clone). For example:
```bash
git clone git@github.com:ECE495/ece495_master_spring2022-USERNAME.git
```
1. Update your git email address and the last naem for you and your team mate:
```
git config --global user.email "you@example.com"
git config --global user.name "Lastname1 Lastname2"
```

## Clone **Robot** repository
1. On the **Master**, open the **robot** GitHub repository and copy your repo address using the SSH mode (same as above).
1. Create a secure shell connection to your robot:
```bash
ssh pi@robotX
```
1. Browse to your workspace source folder:
```bash
cd ~/robot_ws/src/
```
1. Clone your repo using the username and password used when you generated the SSH key (pasting the copied address above after clone). For example:
```bash
git clone git@github.com:ECE495/ece495_robot_spring2022-USERNAME.git
```
1. Update yoru git email address and the last name for you and your team mate.
```
git config --global user.email "you@example.com"
git config --global user.name "Lastname1 Lastname2"
```

## Repository management

It is important that you always pull when you start working on your code on one system and push when you are done working on code on that system.

1. Within a terminal on your **Master**, pull your repo from the `ece495_master_spring2022-USERNAME` folder:
```bash
cd ~/master_ws/src/ece495_master_spring2022-USERNAME
git pull
```

1. Complete work on the **Master**. For example, accomplish the following:
    1. Create a file in your repo (from the `ece495_master_spring2022-USERNAME` folder):
    ```bash
    touch README.md
    nano README.md
    ```
    1. Copy the following to the file completing with your own information:
    ```markdown
    # ECE495 Fundamentals of Robotics
    ## Master System
    This repository stores all code for the master system

    ### Team Member 1
    Name:
    Go by name:
    Hometown:
    Desired AFSC:
    Clubs/IC Sports:
    ### Team member 2
    Name:
    Go by name:
    Hometown:
    Desired AFSC:
    Clubs/IC Sports:
    ```
    1. Hit `ctrl+s` then `ctrl+x` to save and exit the editor.

1. Add the files that will be uploaded to your repository:
```bash
git add -A
```

1. Commit your changes to the repository with a message:
```bash
git commit -m "Completed README on the master!"
```

1. Push your changes to the repository:
```bash
git push
```

1. Create an SSH connection to your **Robot** and pull your repo from the `ece495_robot_spring2022-USERNAME` folder:
```bash
ssh pi@robotX
cd ~/robot_ws/src/ece495_robot_spring2022-USERNAME
git pull
```

1. Complete work on the **Robot**. For example, accomplish the following:
    1. Create a file in your repo (from the `ece495_robot_spring2022-USERNAME` folder):
    ```bash
    touch README.md
    nano README.md
    ```
    1. Copy the following to the file copmleting with your own information:
    ```markdown
    # ECE495 Fundamentals of Robotics
    ## Master System
    This repository stores all code for the robot system

    ### Team Member 1
    Name:
    Go by name:
    Hometown:
    Desired AFSC:
    Clubs/IC Sports:
    ### Team member 2
    Name:
    Go by name:
    Hometown:
    Desired AFSC:
    Clubs/IC Sports:
    ```
    1. Hit `ctrl+s` then `ctrl+x` to save and exit the editor.

1. Add the files that will be uploaded to your repository:
```bash
git add -A
```

1. Commit your changes to the repository with a message:
```bash
git commit -m "Completed README on the robot!"
```

1. Push your changes to the repository:
```bash
git push
```

> ⚠️ **IMPORTANT:** Ensure you pull changes, do work, and push your changes **Every Time** you work on the master or robot!

## Checkpoint
Show the instructor both repositories with README.md files on the GitHub website.
