# Welcome and Shortcuts


Welcome to ECE495: Fundamentals of Robotics Research! To get started, navigate to the "Getting Started" page found on the left hand side of this site.

## Shortcuts

Below you will find some useful shortcuts you can use after completing all the steps in the "Getting Started" page. You may find these useful over the duration of this class.

### 1. Opening a New Terminal

To open a new terminal press ```ctrl```+```alt```+```t``` on your keyboard.


### 2. Opening the Class Website from a New Terminal

To open the class website, use the below command in a new terminal:

```bash
firefox ~/Desktop/ECE495/_build/html/welcome.html
```

### 3. Opening Jupyter Notebook

To open Jupyter Notebook, open a new terminal and navigate to the class files with the command:

```bash
cd Desktop/ECE495
```

You can now run Jupyter Notebook with the command:

```bash
jupyter notebook
```

### 4. Updating your class files

If you mess up your local class files in Jupyter Notebook, you'll need to update your class files. Unfortunately, the best way to do this right now is to go through the "Getting Started" page instructions again. A quick guide of commands to do so are below:

In a new terminal, run each of the following commands:

```bash
cd Desktop
```

```bash
rm ECE495
```

```bash
git clone --depth 1  https://github.com/AF-ROBOTICS/usafabot ECE495
```

```bash
cd ECE495
```

```bash
git filter-branch --prune-empty --subdirectory-filter ECE495 HEAD
```

Everything is now updated!