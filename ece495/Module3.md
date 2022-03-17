# Module 3: Python3 for Robotics
## Rock Paper Scissors User
---
### A note on this document
This document is known as a Jupyter notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Purpose
This assignment will help refresh Python3 fundamentals through application. You will implement a rock paper scissors player and computer that will compete over ROS.

### Import modules at top of Python code
With Python code, the convention is to import all required modules at the top of the python script. Modules are packages of additional functionality built by others. Typically, a module is comprised of compiled code that serves a specific function.

By importing all modules at the beginning of the script, we ensure that all modules have been loaded by the time we need them later in the script. Also, it ensures anyone reading our code can very quickly see all the modules (dependencies) needed to run the code. 

Our imports are in the following code block. 

**Important**: Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules
import random
```

Here, the module is called "random", and by importing the module this way, we can use the module by referencing "random" in our code. There are a few other ways to do imports, but we will wait get to those in another script. 

Another thing to note is we can make comments in python by starting a line with "#". Comments are lines in your code that do not perform any function and are intended only to inform others (and your future self) about the purpose of your code. It is a good idea to document the purpose of your code at the beginning of the code with comments and to leave short and concise comments throughout your code. Many people get lazy and fail to comment their code, but don't fall prey to that trap! 

### Variables
In programming languages, variables are containers that store simple information. In Python, variables can store strings or numbers. A string is a block of text, such as "my USAFABot is faster than yours". A number can be an integer or a decimal number.


```python
# create a variable to hold a string asking the user if they want to play rock, paper, scissors
welcome_text = 'Would you like to play rock, paper, scissors?'

# create a variable holding a number representing the number of possible choices in rock, paper, scissors
num_choices = 3

# verify the variables are holding the correct information by using the "print" function to display the content of both variables
print('The variable welcome_text contains: %s' % welcome_text)
print('The variable num_choices contains: %02d' % num_choices)
```

    The variable welcome_text contains: Would you like to play rock, paper, scissors?
    The variable num_choices contains: 03


A few things are going on here while creating variables. First, Python lets us define variables using the convention variable_name = value. As long as the variable name is unique (it is not the name of a function, module, or other variable), we can define our variable this way. For strings, single quotes or double quotes must be used around the text that needs to be in the variable; it doesn't matter whether you choose single or double quotes, just stay consistent.

Second, we show the print function. The print function takes a string as its input and prints the string to the output, which is right below the block of text. Given `format % values`, *format* includes a string and `%` conversion **items** which are replaced with elements of *values* (such as a 2 digit value `%02d`). The *values* must be a tuple with the exact number of **items** specified in the *format* string.

### Lists
In Python, we can store information in more than just variables. Python has a data type called "lists", and they store just that - lists. An example of a Python list is below.


```python
# create a list containing all of the possible choices in rock, paper, scissors
choices = ['Rock','Paper','Scissors']

# print the contents of "choices" to see what it contains
print(choices)
```

We chose to use strings for each item in the list above, but we could have just as easily chosen numbers or other objects instead. Lists always begin with an open hard bracket, [, and end with the closed hard bracket, ]. Additionally, list items are separated by commas. For this case, choices is the name of our list. Python has other data structures, such as dictionaries, tuples, and classes, but we will stick to variables and lists for now.

You're probably thinking, "Lists look great and all, but how do I use them?" Great question! All of the items in the list are given a position, and in Python lingo, this is called an index. Each item (string in this case) in the list has an index, and the indices start at 0 on the furthest left and increase as you move to the right through the list. You can use an item's index to retrieve it from the list. Using the name of the list, choices in this case, followed by hard brackets [] with the index of the item in the middle will retrieve the item.



```python
# retrieve each item from the list with its index and print it to the output
print('The first item is: %s' % choices[0])
print('The second item is: %s' % choices[1])
print('The third item is: %s' % choices[2])
print('All items of the list include: %s' % choices)

# find the number of items in the list (length of the list)
len(choices)
```

### Functions
In Python, we can create functions to perform a specific task. Functions help us organize our code and simplify our code by allowing us to call a function multiple times rather than writing the same code multiple times. Python functions start with the keyword `def` followed by the function name, an open parenthesis, any inputs to pass to the function, a close parenthesis, and a colon. When we start writing what the function will do, we need to start on a new line and "tab-in" so that Python knows we are writing within the scope of the function. The function finishes when it gets to the last line of code or it reaches a `return` keyword. Let's write a function that represents the computer randomly selecting either 'Rock', 'Paper', or 'Scissors'.


```python
# create a function that represents the computer selecting either 'Rock',
# 'Paper', or 'Scissors' at random
# inputs: choices, which is a list representing any item the 
#         computer can choose
# outputs: rand_choice, an item chosen at random from the input, 
#          choices
def get_comp_choice(choices_list):
    # use random module imported earlier to select a random index from the list
    rand_index = random.randint(0,len(choices_list)-1)
    # use the index to select an item from the list
    rand_choice = choices_list[rand_index]
    # end the function by returning the computer's choice
    return rand_choice
```


```python
# call the function a few times to test that it works properly
# we are using the list 'choices' from earlier 
choice1 = get_comp_choice(choices)
print('Test Choice 1: ',choice1)
choice2 = get_comp_choice(choices)
print('Test Choice 2: ',choice2)
choice3 = get_comp_choice(choices)
print('Test Choice 3: ',choice3)
```

In the function `get_comp_choice`, we use the `randint` function from the `random` module we imported earlier. The `randint` function accepts as its input a range of integers from which it will select an integer at random. Then, it returns this integer and stores it in the variable `rand_index`. The first input to this function is the smallest possible integer to select, and the second input is the largest possible integer to select. The random integer is used to retrieve an item from the input list `choices_list`. Finally, we use the `return` keyword to allow the user to store the choice in a variable when the function is called.

The second code block calls the function 3 times to test that it works as we expect. Press `Shift + Enter` in the code block multiple times to see how the "Test Choices" change. As you can see above, we call a function by using its name followed by an open parenthesis, any inputs, and a close parenthesis. You can pass a different list to the function to further test that it works as you expect! 

### User input
In order to play rock, paper, scissors, we need to get input from the user. The python `input` function is perfect for this. 


```python
# collect an input string from the user 
choice = input("Please choose 'Rock', 'Paper', or 'Scissors' by typing either one of the three:")
```


```python
# print the contents of the variable choice 
print(choice)

# print the type of data contained in the variable choice
print(type(choice))
```

The `input` function takes a string of instructions as input and requests that the user input a string. After the user inputs a string, the function assigns that input string to a variable. In this case, the variable `choice` will contain whatever *string* the user types in the box (numbers typed in the input box will be interpreted as strings).

In the string of instructions passed to the `input` function, the string is surrounded by double quotes, and the individual items are surrounded by single quotes. When placing quotation marks inside of a string, the quotation marks inside the string must be single if the outside quotes are double and vice versa. Lastly, the string includes `\n`, the `\n` tells python to create a newline in the output, and this ensures that the box for input is on a separate line than the instructions. 

We chose to print out the contents of `choice` to the output to show what is actually stored in the variable `choice`. Also, we use the `type` function inside of the `print` function to print the data type being stored in choice, and the output of `<class 'str'>` confirms that strings are always stored in variables created by the `input` function. Try putting different words and numbers in the input box to verify this yourself!

### Validating user input with if statements

Now that we have the user input, let's verify that the user actually did input either Rock, Paper, or Scissors (humans make mistakes after all). We can do this with if statements in Python. If the user inputted a valid string, we can continue with the game. If not, the user has to start over and put in a new string. The key words `if`, `elif`, and `else` followed by a logical expression and a colon denote valid if, else if, and else statements, respectively. Below is an example of an if statement to check if the input is valid.


```python
# use if statement to check if user's input is valid
# start by checking if choice stores the string Rock
if choice == 'Rock':
    print('Good input')
    valid_input = True
#check if choice stores the string Paper
elif choice == 'Paper':
    print('Good input')
    valid_input = True
#check if choice stores the string Scissors
elif choice == 'Scissors':
    print('Good input')
    valid_input = True
# any other strings in choice are invalid
else:
    print('Bad input. Try again.')
    valid_input = False
```

Logical expressions in Python compare one value or string to another. Python uses `==` to test equality and `!=` to test if something is not equal to something else. Python can check if a value is greater or less than another number as well. Googling 'Logical operators in Python' will give a complete list. 

Variables can also store boolean values, meaning they can store `True` or `False`. These are shortcuts for saying something is on or off, and in this case, we are using `True` if the input is valid and `False` if the input is not valid. We will use them later to make our code more concise. 

Can you think of a way to rewrite the if statement above without the `elif` statements? Hint: Python uses `or` and `and` to combine multiple logical expressions into one. Hint for more advanced way: you can use the list `choices` from above. 

One final note about our input validation: if the user inputs rock, what happens? Can you think of a way to accept this as a valid input without adding another `elif` statement? 

### Classes
We will use ROS to communicate and classes enable us to bundle data and functionality together while taking advantage of ROS tools. A class creates new types of objects, allowing *instances* of that class to be made. Each class can have attributes and methods that, combined, maintain and modify a class's state. For example, we could create a `Robot` class that has attributes such as number of wheels, speed, and physics. There might be methods that provide a controller information about the robot's state, such as `get_speed` or `set_speed`. If we were using multiple robots we could have multiple instances of the class and each instance's attributes and methods would only control that particular robot. We will use classes throughout this course.

A class name should use the CapWords convention. When we are operating within the scope of a class, we have to make sure to use the correct namespaces. Within a class you will see the keyword `self` quite a bit. This refers to an instance variable or method and is scoped within the class. For example, if a class has an instance variable `num_wheels`, when within that class, we must access it using the explicit scope: `self.num_wheels`.

Let's discuss the components of the player class

#### Create the player class and init method
```python
class Player:
    # class constant to store player choices
    CHOICES = ['Rock', 'Paper', 'Scissors']
    
    # initialize class
    def __init__(self, num_rounds = 3):
        # instance variables
        self.num_rounds = num_rounds # total numer of games to be played
        self.round = 0 # number of rounds played so far
        self.players_choice = String()
        self.computers_choice = String()
        self.round_complete = True
        self.user_wins = 0
        self.computer_wins = 0
        
        # publisher to send player's choice over the player topic
        self.pub = rospy.Publisher('player', String, queue_size=1)
        
        # timer to request user's input, should be called once/second, but will stall
        # on user input
        rospy.Timer(rospy.Duration(1), self.callback_players_choice)
        
        # subscriber to receive the computer's choice over the computer topic
        rospy.Subscriber('computer', String, self.callback_computers_choice)
```

The above creates and initializes a class called Player. `CHOICES` is a class level variable which is the same for every instance of the class. When an instance of the class is created the number of rounds to be played can be passed to the init function and becomes part of the instance of the class. If no input is provided, the default is set to 3. It is always good to set a default. You can see that comments are provided to describe a few of the instance variables that might need some clarification, but not for the self explanatory variables. That is one benefit of using descriptive variable names, it is easier for some to understand what is going on.

To access an instance variable within the class you must use the `self` keyword.

The last three lines create the ROS publisher, timer, and subscriber. The publisher and subscriber should be familiar to you by now, and the timer periodically calls a callback which we will use to take input from the user. It runs every duration, which is every one second in this example.

#### ROS callback method part of the Player class called by timer
```python
# method to request user input and publish to computer
# inputs: event, ROS information on timing of interrupt.
#         Can be ignored.
# outputs: publishes player's choice
def callback_players_choice(self, event):
    # check to ensure we wait for the round to be complete to get user input
    if self.round_complete and not self.is_game_complete():
        self.players_choice = input('Please choose between %s by typing either one of the three.\n' % self.CHOICES)

        # check to ensure it is a valid input
        if self.players_choice in self.CHOICES:
            self.round_complete = False
            self.pub.publish(self.players_choice)
        else:
            print('Invalid input. Please choose between %s by typing either one of the three.\n' % self.CHOICES)
```

This is an example of a class method. In ROS we call it a callback, because some other process triggers the calling of the method instead of the main function. This particular method is called by the timer. The check is a more concise version of the one we saw earlier. If the check is valid the `choice` *String* is published over the *player* topic.

#### ROS callback method part of Player class called by topic
```python
# method to receive computer's choice and determine winner
# inputs: choice, computer's choice sent over the computer
#         topic
# outputs: none
def callback_computers_choice(self, choice):
    self.computers_choice = choice.data
    self.round += 1
    print('Game %d: %s vs. %s' % (self.round, self.players_choice, self.computers_choice))

    # Determine winner and increase number of wins for player by 1
    if self.players_choice == self.computers_choice: 
        print('Draw')
    elif self.players_choice == 'Rock' and self.computers_choice == 'Scissors':
        print('You win!')
        self.user_wins += 1
    elif self.players_choice == 'Paper' and self.computers_choice == 'Rock':
        print('You win!')
        self.user_wins += 1
    elif self.players_choice == 'Scissors' and self.computers_choice == 'Paper':
        print('You win!')
        self.user_wins += 1
    else:
        print('You lose.')
        self.computer_wins += 1

    self.round_complete = True
```

This class method is another callback that is called whenever a *String* message is sent over the *computer* topic. When called, the message is passed to the module. In this example we named the variable choice and store it in the `computers_choice` instance variable.

#### Class methods part of Player class
```python
# returns the number of wins by the user and computer
def get_results(self):
    return self.user_wins, self.computer_wins

# checks if the game is complete
def is_game_complete(self):
    if self.num_rounds == self.round and self.round_complete:
        return True
    else:
        return False
```

These are two class methods that are used either within the class or in main. Class variables are technically accessible in main, but the first method above is a better convention for accessing instance variables.

### Let's start the game!
Alright, now that we remember the basics of a Python, let's build our game. We will start with the user player.

#### Initialize ROS:
The first step when utilizing  ROS is to initialize *roscore*. There are two methods to accomplish this: first, by explicitly running *roscore* and second, by running a launch file (which will initialize *roscore* if it is not already running). During the first portion of this course we will explicitly run *roscore* and then take advantage of launch files later in the course.

Copy the following code and run it in a new terminal (use the shortcut `ctrl+alt+t` to open a new terminal window):

`roscore`

#### Import modules:


```python
import rospy
from std_msgs.msg import String
```

Here, we are importing two modules: *rospy*, which allows us to run ROS code in Python, and the *String* message from the *std_msgs* ROS package.

#### Build the class:
The below is a copy of what we saw above.


```python
class User:
    # class constant to store user choices
    CHOICES = ['Rock', 'Paper', 'Scissors']
    
    # initialize class
    def __init__(self, num_rounds = 3):
        # instance variables
        self.num_rounds = num_rounds # total numer of games to be played
        self.round = 0 # number of rounds played so far
        self.users_choice = String()
        self.computers_choice = String()
        self.round_complete = True
        self.user_wins = 0
        self.computer_wins = 0
        
        # publisher to send user's choice over the user topic
        self.pub = rospy.Publisher('user_choice', String, queue_size=1)
        
        # timer to request user's input, should be called once per second, but will stall
        # on user input
        rospy.Timer(rospy.Duration(1), self.callback_users_choice)
        
        # subscriber to receive the computer's choice over the computer topic
        rospy.Subscriber('computer_choice', String, self.callback_computers_choice)
            
    # method to request user input and publish to computer
    # inputs: event, ROS information on timing of interrupt.
    #         Can be ignored.
    # outputs: publishes user's choice
    def callback_users_choice(self, event):
        # check to ensure we wait for the round to be complete to get user input
        if self.round_complete and not self.is_game_complete():
            self.users_choice = input('Please choose between %s by typing either one of the three.\n' 
                                      % self.CHOICES)

            # check to ensure it is a valid input
            if self.users_choice in self.CHOICES:
                self.round_complete = False
                self.pub.publish(self.users_choice)
            else:
                print('Invalid input. Please choose between %s by typing either one of the three.\n' 
                      % self.CHOICES)
                
    # method to receive computer's choice and determine winner
    # inputs: choice, computer's choice sent over the computer
    #         topic
    # outputs: none
    def callback_computers_choice(self, choice):
        self.computers_choice = choice.data
        self.round += 1
        print('Game %d: %s vs. %s' % (self.round, self.users_choice, self.computers_choice))
        
        # Determine winner and increase number of wins for user by 1
        if self.users_choice == self.computers_choice: 
            print('Draw')
        elif self.users_choice == 'Rock' and self.computers_choice == 'Scissors':
            print('You win!')
            self.user_wins += 1
        elif self.users_choice == 'Paper' and self.computers_choice == 'Rock':
            print('You win!')
            self.user_wins += 1
        elif self.users_choice == 'Scissors' and self.computers_choice == 'Paper':
            print('You win!')
            self.user_wins += 1
        else:
            print('You lose.')
            self.computer_wins += 1
        
        self.round_complete = True

    # returns the number of wins by the user and computer
    def get_results(self):
        return self.user_wins, self.computer_wins
    
    # checks if the game is complete
    def is_game_complete(self):
        if self.num_rounds == self.round and self.round_complete:
            return True
        else:
            return False
```

#### Create the main


```python
def main():
    # initialize a node called player
    rospy.init_node('user')

    print("***Make sure your computer player is running.***")

    num_rounds = int(input('How many rounds do you want to play? '))

    # create an instance of the class that will play the specified number of games
    u = User(num_rounds)

    # run continuously until the game is complete
    while not u.is_game_complete():
        pass

    # print results
    user_wins, computer_wins = u.get_results()
    print('Total number of user wins: %s' % (user_wins))
    print('Total number of computer wins: %s' % (computer_wins))
    print('Total number of draws: %s' % (num_rounds - (user_wins + computer_wins)))
    print('User win percentage: %s' % (user_wins/num_rounds))
```

In an actual Python script we will replace 
```python
def main()
``` 
with 
```python
if __name__ == "__main__":
```
This allows our python files to be imported into other python files that might also have a main() function.

#### Create the computer player
We must build and run the computer player before we can continue. Open the [Module3_RPSComputer](Module3_RPSComputer.ipynb) notebook and follow the instructions.

#### Run our User class


```python
main()
```

### ROS refresher

Open the [Module3_ROS](Module3_ROS.ipynb) notebook and follow the instructions.


## ROS
Below you will see some ROS commands. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the user and computer nodes running execute the below commands.

List the active topics:


```python
!rostopic list
```

Display running nodes and communication between them:


```python
!rqt_graph
```

Exit the rqt_graph and then list all running nodes:


```python
!rosnode list
```

Show information about a the **/computer_choice** topic such as what type of messages are sent over the topic and publishing and subscribing nodes:


```python
!rostopic info /computer_choice
```

Display information about the message that is sent over the **/computer** topic:


```python
!rostopic type /computer_choice | rosmsg show
```

Display messages sent over the **/user_choice** topic:


```python
!rostopic echo /user_choice
```

In the Module3_Python3 notebook, reset the Jupter kernel and clear output: At the top menu bar select "Kernel" and "Restart & Clear Output" (you do not need to restart the computer player). Make sure to rerun the code blocks to import modules, build the class, and create the main. When you select 'Rock', 'Paper', or 'Scissors' you should see the message echoed above.

### Cleanup
In each of the notebooks reset the Jupter kernel and clear output. Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `Jupter-notebook` in. Select 'y'. Kill all processes in your terminals (ctrl+c) and ensure roscore is terminated before moving on to the ICE.

You should now have a better understanding of how to utilize ROS and Python. To learn more about the Python style guide and standards, visit [PEP 8 -- Style Guid for Python Code](https://www.python.org/dev/peps/pep-0008/#class-names).




## Rock Paper Scissors Computer
---
### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Computer Player
We will now implement a computer player node that will send a random choice to the player node.

First import your required modules:


```python
import rospy, random
from std_msgs.msg import String
```

Here, we are importing two modules: *rospy*, which allows us to run ROS code in Python, and the *String* message from the *std_msgs* ROS package.


```python
class Computer:
    # class constant to store computer choices
    CHOICES = ['Rock', 'Paper', 'Scissors']
    
    # initialize class
    def __init__(self,):
        # instance variables
        self.computers_choice = String()
        
        # subscriber to receive the computer's choice over the computer topic
        rospy.Subscriber('user_choice', String, self.callback_computers_choice)
        
        # publisher to send player's choice over the player topic
        self.pub = rospy.Publisher('computer_choice', String, queue_size=1)
    
    def callback_computers_choice(self, data):
        # use random module imported earlier to select a random index from the list
        rand_index = random.randint(0,len(self.CHOICES)-1)
        # use the index to select an item from the list
        self.computers_choice = self.CHOICES[rand_index]
        self.pub.publish(self.computers_choice)
```


```python
rospy.init_node('computer')
Computer()
rospy.spin()
```

Now return to the ICE3_RPSUser Notebook and fill in the requested inputs.


## In-Class Exercise 3 - Client
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE3_Client.ipynb)


### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Purpose
This Jupyter Notebook will allow you to practice some of the techniques you have learned over the last few modules. You will develop an advanced chat client and server (similar to ICE1) enabling a client to send a message and a server to respond accordingly.

#### Initialize ROS:
The first step when utilizing  ROS is to initialize *roscore*. There are two methods to accomplish this: first, by explicitly running *roscore* and second, by running a launch file (which will initialize *roscore* if it is not already running). During the first portion of this course we will explicitly run *roscore* and then take advantage of launch files later in the course.

Copy the following code and run it in a new terminal (use the shortcut `ctrl+alt+t` to open a new terminal window or select an open terminal and hit `ctrl+shift+t` to open a new terminal tab):

`roscore`

### Implementing the chat client
**NOTE**: This Jupyter Notebook will require you to enter Python3 code within code sections. You can type any Python3 code and expand the block if necessary. After typing the code, execute the code block before moving forward.

#### Import modules

**Important**: Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules for ROS and the String message from std_msgs


```

#### Client Class
1. Create a Client class with a dictionary used to map numbers to messages.
2. Initialize the class with the following:
    1. an instance variable to store the String message
    2. a publisher that publishes String messages on the client topic
    3. a subscriber to the server topic which receives String messages and calls a callback when messages are sent.
    4. a timer that runs every second and calls a class method
    5. nicely handle shutdown
3. Create the callback input class method that is ran every second and has the user pick a message to send.
4. Create the callback received class method that is called every time a message is received from the server.
5. Handle shutdown.


```python
class Client:
    MESSAGE = {1: "Hello!", 2: "How are you?", 3: "Where are you from?",
               4: "What are you doing today?"}
    
    def __init__(self):
        # 2.A

        # 2.B
        
        # 2.C
        
        # 2.D
        
        # 2.E nicely handle shutdown
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def callback_input(self, event):
        valid = False
        while not valid and not self.ctrl_c:
            # get input from user (you must inform them their options)
            
            try:
                # convert to int, if not number throw ValueError
                val = int(chat_str)
                # check if valid number, if valid then access
                # that entry in the dictionary, publish the message
                # and set valid to True; if not valid, print error
                # message to user (make this error message useful)
                
                
                
                
                
            except ValueError:
                # print error message to user (make 
                # this error message useful)
                
                
    def callback_received(self, msg):
        # print message sent to the server
        
        # print the response from the server
    
    # handle shutdown
    def shutdownhook(self):
        print("Shutting down the client")
        self.ctrl_c = True
```

#### Main
The main function calls initializes our node, creates an instance of the client class, then runs forever.


```python
def main():
    # initialize node
    
    # create an instance of the client class
    
    # run forever
    rospy.spin()
```

#### Run the client


```python
main()
```

#### Create the server
At this point the client is waiting for user input. Don't start typing yet, though! We need to implement and run our server. Open the [ICE3_Server](ICE3_Server.ipynb). notebook and follow the instructions.

### ROS commands
Open the [ICE3_ROS](ICE3_ROS.ipynb) notebook and follow the instructions.


## In-Class Exercise 3 - Server
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE3_Server.ipynb)


### Implementing the chat server
**NOTE**: This Jupyter Notebook will require you to enter Python3 code within code sections. You can type any Python3 code and expand the block if necessary. After typing the code, execute the code block before moving forward.

#### Import modules

**Important**: Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules for ROS and the String message from std_msgs


```

#### Server Class
1. Create a server class with a dictionary used to map messages to responses (one for each message from the client).
2. Initialize the class with the following:
    1. an instance variable to store the String message
    2. a subscriber to the client topic which receives String messages and calls a callback called received.
    3. a publisher to the server topic which sends String messages 
    4. nicely handle shutdown
3. Create the callback received class method that is called every time a message is received from the client.
5. Handle shutdown.


```python
class Server:
    # class dictionary storing server responses
    MESSAGE = 
    
    def __init__(self):
        # 2.A

        # 2.B
        
        # 2.C
        
        # 2.D nicely handle shutdown
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def callback_received(self, msg):
        # print the message from the client
        
        # print the response that will be sent to the client
        
        # publish the response
    
    # handle shutdown
    def shutdownhook(self):
        print("Shutting down the server")
        self.ctrl_c = True
```

#### Main
The main function calls initializes our node, creates an instance of the server class, then runs forever.


```python
def main():
    # initialize node
    
    # create an instance of the server class
    
    # run forever
    rospy.spin()
```

#### Run the server


```python
main()
```

At this point, the server is waiting for the client to send a message. Browse back to your client and type a message! You should see that message show up above.

## In-Class Exercise 3 - ROS
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE3_ROS.ipynb)


Below you will run ROS commands. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the client and server nodes running execute the below commands.

List all running nodes:


```python
!
```

List the active topics:


```python
!
```

Display running nodes and communication between them:


```python
!
```

Exit the rqt_graph.

Show information about a the **/client** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.


```python
!
```

Display information about the message that is sent over the **/client** topic.


```python
!
```

Display messages sent over the **/client** topic:


```python
!
```

In the ICE3_Client notebook when you send a message you should see the message echoed above.

### Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

### Cleanup
In each of the notebooks reset the Jupter kernel and clear output. Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.