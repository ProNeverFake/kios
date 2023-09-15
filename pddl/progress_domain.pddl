(define (domain fluent_domain)
  (:requirements :strips :typing :fluents :durative-actions :duration-inequalities)
  
  (:types 
    Product Robot Location - Object
    MobileRobot PandaRobot - Robot
    InsertionStation - Location
  )

  (:predicates
    ; robot common properties
    (isRobotFree ?robot - Robot)
    
    (isRobotatLocation ?robot - Robot ?location - Location)
    (isRobotatInsertionstation ?robot - Robot ?station - InsertionStation)

    ; panda robot ppt
    (hasRobotRecovered ?panda_robot - PandaRobot)
    ; *
    (isPerforming ?panda_robot - PandaRobot)
    (hasFinished ?panda_robot - PandaRobot)

    ; mobile robot ppt
    (isRobotIdle ?robot - Robot)

    ; product ppt
    (isProductatRobot ?product - Product ?robot - Robot)
    (isProductAssembled ?product - Product ?location - Location)
    
  )

  (:functions
  (WorkProgress ?panda_robot - PandaRobot) ; in percentage
  ; ... other functions
)
  
  ; common work start action
  (:action InsertionStart
    :parameters (?mobile_robot - Robot ?panda_robot - PandaRobot ?station - Location ?product - Product)
    :precondition (and 
                  (isRobotatLocation ?mobile_robot ?station)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?panda_robot ?station)
                  (isRobotIdle ?mobile_robot)
                  (hasRobotRecovered ?panda_robot)
               )
    :effect (and 
              (isPerforming ?panda_robot)
              (not (isRobotIdle ?mobile_robot))
              (not (hasRobotRecovered ?panda_robot))

            )
  )

  (:durative-action Inserting
  :parameters (?panda_robot - PandaRobot)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (and
                  ; (at start (hasRobotRecovered ?panda_robot))
                  (at start (isPerforming ?panda_robot))
                  (at start (< (WorkProgress ?panda_robot) 100))
                )
  :effect (and
            ; (at start (not (hasRobotRecovered ?panda_robot))) 
            (at start (not (isPerforming ?panda_robot)))
            (increase (WorkProgress ?panda_robot) (* #t 1)) ; increase progress by 10%
            (at end (isPerforming ?panda_robot))
            ; (at end(increase (WorkProgress ?panda_robot) 10))
           )
  )

  (:action InsertionFinish
  :parameters (?mobile_robot - Robot ?panda_robot - PandaRobot ?product - Product ?location - Location)
  :precondition (and
                  (>= (WorkProgress ?panda_robot) 100)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?mobile_robot ?location)
                )
  :effect (and
            (assign (WorkProgress ?panda_robot) 0)
            (isProductAssembled ?product ?Location)
            (not (isPerforming ?panda_robot))
            (hasFinished ?panda_robot)
            (isRobotIdle ?mobile_robot)
           )
  )



  ; just recover the pandarobot from the last task
  (:durative-action Recover
    :parameters (?panda_robot - PandaRobot)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (hasFinished ?panda_robot))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )

  (:durative-action Move
    :parameters (?mr - MobileRobot ?from - Location ?to - Location)
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


