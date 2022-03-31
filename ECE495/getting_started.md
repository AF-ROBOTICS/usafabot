# Getting Started


## 1. Cloning the GitHub Repository

You will need to download the entire GitHub repository for this class.

**Using Git**

In a new terminal, create a new folder on your desktop:

```bash
cd Desktop
```

Now download the files from GitHub with the command:

```bash
git clone --depth 1  https://github.com/AF-ROBOTICS/usafabot ECE495
```

The above command made a new directory on your desktop. Navigate to the new folder you just created. We will be using this to hold all the 495 files you'll need for class.

```bash
cd ECE495
```

Next, we will clean up the folder so it has the correct paths with the command below.

```bash
git filter-branch --prune-empty --subdirectory-filter ECE495 HEAD
```

You now have all the files you will need for class. Do not change the folder's name on your desktop or the files you will need for class will not work correctly.


## 2. Opening the Class Website

Once you've completed all of the steps above, you should be able to open the class website with the below command in a **new terminal**:

```bash
firefox ~/Desktop/ECE495/_build/html/welcome.html
```

We will now download jupyter notebook.


## 3. Opening Jupyter Notebook

To install Jupyter Notebook, open a **new terminal** and use the command:

```bash
pip3 install jupyter
```

Once you've completed all of the steps above, you should be able to run Jupyter Notebook.

To test it out, open a new terminal and navigate to the ECE495 folder: 

```bash
cd Desktop/ECE495
```

We will now open the Jupyter Notebook. In Jupyter Notebook, you will be able to edit the ECE495 files on your local computer. Be careful however, we will need these files for class. To open Jupyter Notebook. Run the command:

```bash
jupyter notebook
```