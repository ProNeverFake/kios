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
      :parameters (?e - (either fruit apple))
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
