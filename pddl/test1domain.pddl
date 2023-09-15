(define (domain test1_domain)
  (:requirements :strips :typing :fluents :durative-actions)
  
  (:types 
    veg fruit - object
    apple - fruit
    tomato - veg
  )

  (:predicates
      (isapple ?a - apple)
    (isfruit ?f - fruit)

      (testpredicate)
  )

  (:durative-action check
      :parameters (?e - fruit)
      :duration (= ?duration 1)
      :condition (and 
            ; (at start(isapple ?e))
            (at start (isfruit ?e))
      )
      :effect (and 
        (at end (testpredicate))
        )
  )

)

Hello everyone! I'm now creating the domain file and the problem file for an assembly problem and I want to use plansys2 in ROS2 to make the robots execute the plan. Now I'm using the popf ROS2 package and there is a solver-dependent problem: the subtype cannot be taken as its supertype in actions' parameter.
