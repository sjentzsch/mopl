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
-- Location: this mopl git repository folder
-- Project Name: mopl
-- Settings -> C/C++ Build -> "make -j8" for all configurations, and uncheck "generate makefiles automatically"
-- In the run configurations (create one for each 'Release' and 'Debug'):
--- Application: e.g. "Release/src/rlDamaDemoGUI/rlDamaDemoGUI"
--- choose the respective Build configuration
--- Arguments -> Working directory: ${workspace_loc:mopl}/output

Testing:
--------------
- Within your git repository root folder, execute: "cd Release && make -j8 && ctest"
- If all test scenarios run nice and smoothly, you are ready to start your own MOPL-Experiments!
