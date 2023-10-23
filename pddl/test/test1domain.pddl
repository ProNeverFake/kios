(define (domain test1_domain)
  (:requirements :strips :typing :fluents :durative-actions :duration-inequalities)
  
  (:types 
    veg fruit location - object
    apple pear - fruit
    tomato some - veg
    placea placeb - location
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
