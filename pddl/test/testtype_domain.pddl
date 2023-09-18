(define (domain type_domain)
  (:requirements :strips :typing :fluents :durative-actions)
  
  (:types 
    location robot product - object
    InsertionStation screwingstation - location
    mobilerobot - robot
    productsp cylinder bolt - product
  ) 

  (:predicates  
    ; robot common properties
    (isRobotatLocation ?robot - robot ?location - location)
    (isinsertionstation ?station - insertionstation)
    (isscrewingstation ?station - screwingstation)
    ; mobile robot ppt
    (isRobotIdle ?robot - robot)
  )

  ; * TYPE SAFE
  (:durative-action Move
    :parameters (?mr - mobilerobot ?from - location ?to - location)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (isRobotatLocation ?mr ?from))
                  (at start (isRobotIdle ?mr))
               )
    :effect (and 
              (at start (not (isRobotIdle ?mr)))
              (at start (not (isRobotatLocation ?mr ?from)))
              (at end (isRobotatLocation ?mr ?to))
              (at end (isRobotIdle ?mr))
            )
  )

)


