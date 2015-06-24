MOPL: A Multi-Modal Path Planner for Generic Manipulation Tasks
==============

MOPL: A Multi-Modal Path Planner for Generic Manipulation Tasks, inspired by the diverse action manipulation (DAMA) algorithm, and build upon the Robotics Library (RL). 

Installation (for Ubuntu + Eclipse):
--------------
- Assure to have the Robotics Library installed (http://www.roboticslibrary.org/)
- Clone this git repository
- Within the root folder, create subfolders 'Debug', 'Release', and 'output'
- Within each the 'Debug' and 'Release' folder, execute "ccmake ..", press 'c', as CMAKE_BUILD_TYPE enter either "RelWithDebInfo" or "Debug", ignore the warnings, and generate
- In eclipse: New -> C++ Project -> Executable -> Empty Project -> Cross GCC
	- Location: this mopl git repository folder
	- Project Name: mopl
	- Settings -> C/C++ Build -> "make -j8" for all configurations, and uncheck "generate makefiles automatically"
	- In the run configurations (create one for each 'Release' and 'Debug'):
		- Application: "Release/src/rlDamaDemoGUI/rlDamaDemoGUI", resp. "Debug/src/rlDamaDemoGUI/rlDamaDemoGUI"
		- choose the respective Build configuration
		- Arguments -> Working directory: ${workspace_loc:mopl}/output
		- Arguments -> Program arguments: "../data/tasks/kuka-scenario1.xml" (i.e., here you choose the scenario to be solved)

Testing:
--------------
- Within your git repository root folder, execute: "cd Release && make -j8 && ctest"
- If all test scenarios run nice and smoothly, you are ready to start your own MOPL-Experiments!

Running Scenarios:
--------------
- Switch to "Release" build mode, "Debug" is rather slow and should only be used for bugfixing
- In the 'Program arguments' within the 'Release Run Configuration' you can choose which scenario/task you want to solve. Have a look at the folder "data/tasks/" which scenarios are currently available. The currently evaluated scenarios from our MOPL-paper are: "kuka-scenario1.xml", "kuka-scenario2.xml", "meka-scenario1.xml", and "mobile-robot-scenario1.xml".
- Note that tasks are defined in a mark-up language and allow modification of robot and workspace models, sampling spaces, problem description, planner settings, manipulation primitives, sampling and metric parameters, and post processing routines like path smoothing. Furthermore, they also define parameters of the viewer, like the mode (loop|once|off) and the simulation speed.
- For executing a task from the command line, run from the root directory: "cd output && ../Release/src/rlDamaDemoGUI/rlDamaDemoGUI ../data/tasks/mobile-robot-scenario1.xml"
- Within the folder "output/", a csv-file "DamaBenchmark.csv" contains information about each scenario you did run in the past.
