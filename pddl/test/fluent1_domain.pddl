; ! ALL the insertion and move load unload
(define (domain fluent1_domain)
  (:requirements :strips :typing :fluents :durative-actions :duration-inequalities)
  
  (:types 
    Product Robot Location - Object
    MobileRobot PandaRobot - Robot
    InsertionRobot - PandaRobot
    
    Cylinder ProductSP - Product
  )

  (:predicates
    
    ; location
    (isLocationFree ?location - Location)

    ; robot common properties
    (isRobotFree ?robot - Robot)
    
    (isRobotatLocation ?robot - Robot ?location - Location)
    (isRobotatInsertionstation ?robot - Robot ?station - InsertionStation)

    ; panda robot ppt
    (hasRobotRecovered ?panda_robot - PandaRobot)
    ; *
    (isPerforming ?panda_robot - PandaRobot ?product - Product)
    (hasFinished ?panda_robot - PandaRobot ?product - Product)

    ; mobile robot ppt
    (isRobotIdle ?robot - Robot)

    ; product ppt
    (isProductatRobot ?product - Product ?robot - Robot)
    (isProductatLocation ?product - Product ?location - Location)

    (isProductProcessed ?product - Product ?panda_robot - PandaRobot)
    
  )

  (:functions
  (WorkProgress ?panda_robot - PandaRobot ?product - Product) ; in percentage
  ;distance
  (Distance ?from - Location ?to - Location)
  ;mobile_robot velocity
  (Velocity ?mobile_robot - MobileRobot)
)
  
  ; SP
  (:durative-action InsertionStartSP
    :parameters (?mobile_robot - MobileRobot ?panda_robot - InsertionRobot ?station - Location ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (at start(and 
                  (isRobotatLocation ?mobile_robot ?station)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?panda_robot ?station)
                  (isRobotIdle ?mobile_robot)
                  (hasRobotRecovered ?panda_robot)
               ))
    :effect (and 
              (at start (isPerforming ?panda_robot ?product))
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasRobotRecovered ?panda_robot)))
            )
  )

  (:durative-action InsertingSP
  :parameters (?panda_robot - InsertionRobot ?product - ProductSP)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (and
                  ; (at start (hasRobotRecovered ?panda_robot))
                  (at start (isPerforming ?panda_robot ?product))
                  (at start (<= (WorkProgress ?panda_robot ?product) 100))
                )
  :effect (and
            ; (at start (not (hasRobotRecovered ?panda_robot))) 
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%
            (at end (isPerforming ?panda_robot ?product))
            ; (at end(increase (WorkProgress ?panda_robot) 10))
           )
  )

  (:durative-action InsertionFinishSP
  :parameters (?mobile_robot - MobileRobot ?panda_robot - InsertionRobot ?product - ProductSP ?location - Location)
  :duration (= ?duration 0.001)
  :condition (at start(and
                  (>= (WorkProgress ?panda_robot ?product) 30)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?mobile_robot ?location)
                ))
  :effect (at end (and

            (assign (WorkProgress ?panda_robot ?product) 0)
            (isProductProcessed ?product ?panda_robot)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
            (isRobotIdle ?mobile_robot)
           ))
  )

  ; just recover the pandarobot from the last task
  (:durative-action InsertionRecoverSP
    :parameters (?panda_robot - InsertionRobot ?product - ProductSP)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )

  (:durative-action InsertionStart
    :parameters (?panda_robot - InsertionRobot ?station - Location ?product - Cylinder)
    :duration (= ?duration 0.001)
    :condition (at start(and 
                  (isProductatLocation ?product ?station)
                  (isRobotatLocation ?panda_robot ?station)
                  (hasRobotRecovered ?panda_robot)
               ))
    :effect (and 
                  (at start (not (hasRobotRecovered ?panda_robot)))
                  (at end (isPerforming ?panda_robot ?product))
            )
  )

  (:durative-action Inserting
  :parameters (?panda_robot - InsertionRobot ?product - Cylinder)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (and
                  (at start (isPerforming ?panda_robot ?product))
                  (at start (<= (WorkProgress ?panda_robot ?product) 100))
                )
  :effect (and
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%
            (at end (isPerforming ?panda_robot ?product))
           )
  )

  (:action InsertionFinish
  :parameters (?panda_robot - InsertionRobot ?product - Cylinder ?location - Location)
  :precondition (and
                  (>= (WorkProgress ?panda_robot ?product) 20)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatLocation ?product ?location)
                )
  :effect (and
            (assign (WorkProgress ?panda_robot ?product) 0)
            (isProductProcessed ?product ?panda_robot)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
           )
  )

  ; just recover the pandarobot from the last task
  (:durative-action InsertionRecover
    :parameters (?panda_robot - InsertionRobot ?product - Cylinder)
    :duration (= ?duration 10)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )

  ; (:durative-action Move
  ;   :parameters (?mr - MobileRobot ?from - Location ?to - Location)
  ;   :duration (= ?duration 5)
  ;   :condition (and 
  ;                 (at start (isRobotatLocation ?mr ?from))
  ;                 (at start (isRobotIdle ?mr))
  ;              )
  ;   :effect (and 
  ;             (at start (not (isRobotIdle ?mr)))
  ;             (at start (not (isRobotatLocation ?mr ?from)))
  ;             (at end (isRobotatLocation ?mr ?to))
  ;             (at end (isRobotIdle ?mr))
  ;           )
  ; )

  ; * TYPE SAFE
  (:durative-action Move
    :parameters (?mr - MobileRobot ?from - Location ?to - Location)
    :duration (= ?duration (/ (Distance ?from ?to) (Velocity ?mr)))
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

  ; ; * TYPE SAFE 
  ; ! DISCARDED
  ; (:durative-action MoveReverse
  ;   :parameters (?mr - MobileRobot ?from - Location ?to - Location)
  ;   :duration (= ?duration (/ (Distance ?to ?from) (Velocity ?mr)))
  ;   :condition (and 
  ;                 (at start (isRobotatLocation ?mr ?from))
  ;                 (at start (isRobotIdle ?mr))
  ;              )
  ;   :effect (and 
  ;             (at start (not (isRobotIdle ?mr)))
  ;             (at start (not (isRobotatLocation ?mr ?from)))
  ;             (at end (isRobotatLocation ?mr ?to))
  ;             (at end (isRobotIdle ?mr))
  ;           )
  ; )

   ; * SAFE
  (:durative-action Load
    :parameters (?mobile_robot - MobileRobot ?product - ProductSP ?location - Location)
    :duration (= ?duration 5)
    :condition (and 
                  (at start (isProductatLocation ?product ?location))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isRobotIdle ?mobile_robot))
                  (at start (isRobotFree ?mobile_robot))
               )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (isProductatLocation ?product ?location)))
              (at end (isRobotIdle ?mobile_robot))
              (at end (isLocationFree ?location))
              (at end (not (isRobotFree ?mobile_robot)))
              (at end (isProductatRobot ?product ?mobile_robot))
            )
  )

  ; * SAFE
  (:durative-action Unload
    :parameters (?mobile_robot - MobileRobot ?product - ProductSP ?location - Location)
    :duration (= ?duration 5)
    :condition (and 
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isLocationFree ?location))
                  (at start (isRobotIdle ?mobile_robot))
              )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (isProductatRobot ?product ?mobile_robot)))
              (at end (isProductatLocation ?product ?location))
              (at end (not (isLocationFree ?location)))
              (at end (isRobotFree ?mobile_robot))
              (at end (isRobotIdle ?mobile_robot))
            )
  )
 

)


