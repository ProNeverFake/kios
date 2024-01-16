
# Unified-planning
https://github.com/ProNeverFake/unified-planning
- A python interface for planning problems.
- pretty off-the-shelf

### Pros
- a PDDL<--->python interface 
- support multiple planners

### Cons
- doesn't have the embedding behavior tree module

### Suggestions
- use its interface to generate and validate PDDL problem files.
- make use of the python-represented plan to save plan parsing effort.

---

# Rosplan
https://kcl-planning.github.io/ROSPlan/documentation/
- TAMP framework based on Ros1
- the dispatcher run the plan as an action sequence. 
- The model keeps a knowledge base and the actions will modify the knowledge base according to its effect (also for checking the preconditions).
### Pros
- provide interfaces for pddl and pddl plan parsers (including an interface to yaml).
- good ros service for the knowledge base in planning.
- good visualization of temporal planning.

### Cons
- no behavior tree support
- bloated plan dispatch part

### suggestions:
- make use of its knowledge base part to make the runtime state update and error detection. But this may be hard because use Ros1 service can not satisfy the BT ticking rate (20Hz).

---

# Skiros2
https://github.com/RVMI/skiros2
- ontology + PDDL + extended behavior tree based skills
- Behavior trees are used implicitly. The behavior tree control flows are only implemented in compound skills. 
- The action model is essentially still a skill chain.

### Pros
- developed in python, also provides a world model.

### Cons
- The world model is described by ontology but not PDDL domain. no interface for PDDL domain.
- extended BT doesn't allow dynamic expanding of condition nodes (since the condition nodes are embedded in the action node.)

### Suggestions:
- use this only if we use ontology.
- The design of world model can be used as reference. But can not directly use it since the lack of PDDL domain interface.


---

# FlexBE
http://philserver.bplaced.net/fbe/
- Behavior Engine for fast robot behavior deployment in ros(2).
- Essentially a hierarchical state machine.
- drag-and-drop robot behavior modification. non-expert-friendly.

### Suggestions:
- Don't use this one.

---

# BehaviorTree.CPP 3.x
https://www.behaviortree.dev/docs/3.8/intro
- The old version of the BT.CPP package.
- used by Plansys2 to represent the actions. 

### Pros
- has visualization "groot". For 4.x this functionality is charged. But still not a runtime visualization.

### Cons
- no reason to use this if using the 4.x version is possible
- XML file is not compatible with the 4.x version

### Suggestions
- use this only if using Plansys2. But still not recommanded.

---

# Plansys2
https://plansys2.github.io/tutorials/docs/bt_actions.html#overview
- A ros2 package for robot task planning and execution.
- Planning part: PDDL. Excuting part: (behavior tree-represented) action chain
- (extended) behavior trees are explicitly implemented with BT.CPP 3.x. However, the action model is still essentially an action chain.

### Pros
- off-the-shelf (from plan to action)
- provide PDDL solver support (POPF, TF, SMT) and a parser
- support parallel and multi-robot action execution

### Cons
- No runtime reaction mechanism
- hard to modify the existing tree becaused of BT.CPP
- still using BT.CPP 3.x
- I didn't find the knowledge base (at least not shown in the tutorials) so I doubt that the plan execution part is pure open-loop. (The precondtions and effects of actions will not be checked or take effect at runtime, but are used for constructing the action sequence.)

### Suggestions
- All its PDDL part can be substituted by [[#Unified-planning]]. 
- The way to assemble the subtrees can be adopted. But I think we need to develop it again in python based on pytrees because BT.CPP hides the native interfaces for BT modification.

---

# BehaviorTree.CPP 4.x
https://www.behaviortree.dev/
- A c++ implementation of behavior tree. All the basic functionality of behavior tree is implemented and works well.
- Some projects use this to manually design a behavior tree for discrete task usages. 

### Pros
- the strengths of cpp.
- nearly a complete implementation in kios now.
- improvements based on BT.CPP 3.x

### Cons
- Hidden interfaces for the direct modification of the tree instance: 
Tree modifying methods are not officially exposed. To modify an old tree, the user needs to modify its original XML file and generate the tree instance again.

- Isolated blackboards: 
The blackboards are all local to the subtrees and need to be remapped in the XML. (Can be solved using a global pointer.)

- No runtime visualization. 
Groot2, the visualization tool of BT.CPP, only supports the visualization of XML file (apis for generating XML for visualization are provided). Runtime node tick visualization is only available for Pro user.

### Suggestions:
- A good library for implementing static behavior trees. In other words, this library should be used for manual (expert) behavior tree designing. 

---

## Py_trees
https://py-trees.readthedocs.io/en/devel/
- A python implementation of behavior tree. All the basic functionality is provided and works well. Basically the same as the C++ implementation above. 
- The behavior tree expanding algorithm is using this.

### Pros
- Direct tree modification methods, highly flexible 
Add, delete or other fine changing modifications can be performed directly to the tree instance. 

- Runtime visualization available.
In tutorials the visualization is like the repository tree. Apis for generating behavior tree in a graph are provided too.

### Cons
- No formal saving file form of the tree instance.
Unlike BT.CPP, it doesn't provide the apis for recording the tree instances in a formal language like XML. The user needs to write his own read/write interfaces for recording/reusing a previous tree instance.

### Suggestions:
- Perfect for the requirements of the dynamics in behavior trees. For the behavior tree expanding method this is the most recommanded library.

---
