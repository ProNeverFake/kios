### About this module

This module utilizes the `unified-planning` package to generate PDDL files, namely domain and problem files, by writing the contents in python. 

At the initial development phase of kios, this module serves as a tool to generate PDDL-based planning tasks, which is then used to test and evaluate the capability of LLMs in solving planning tasks. 

Finally, after the feasibility is confirmed, PDDL-based problems are not in use anymore. One of the reasons is the close-world assumption in PDDL, which may make LLMs confused when solving the tasks. Besides, PDDL is also not so user-friendly, which is also contrary to the usage of behavior trees to improve the interpretability of plans.

Long story short, this module is not in use anymore, but it is still kept in the repository for reference purposes.