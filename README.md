# rt2_ass2 project
Second and final assignment of the Research Track 2 course

# How are all the branches structerd:

- main: code for executing the project 
- doxygen: code for executing the project + doxygen documentation
- sphinx: code for executing the project + sphinx documentation

# How to run the code:

Choose a branch, then: 

If you want the classical gazebo visualization:

- roslaunch rt2_ass2 sim.launch
 
If you don't want to see gazebo opened due to, for example, the amount of cpu usage:
 
- roslaunch rt2_ass2 sim_small.launch
 
To open the user_interface:
  
- Open jupyter notebook, or jupyter lab;
- From one of the two software open "user_notebook_interface.ipynb" located in the folder notebooks

# How to visualize the doxygen documentation:
- open the index.html file in the doc/html folder
