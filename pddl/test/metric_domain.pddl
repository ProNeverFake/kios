(define (domain metric_domain)
  (:requirements :strips :typing :equality :fluents :durative-actions :duration-inequalities :universal-preconditions)
  
  (:types 
    Product Robot Location - Object
    MobileRobot PandaRobot - Robot
    ScrewingRobot InsertionRobot InspectionRobot - PandaRobot
    
    Bolt Cylinder ProductSP - Product
  )

  (:predicates
    
    ; location
    (isLocationFree ?location - Location)

    ; robot common properties
    (isRobotFree ?robot - Robot)
    
    (isRobotatLocation ?robot - Robot ?location - Location)

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
    
    ; product sp ppt
    (isInspected ?product - ProductSP)
    (isAssembled ?product - ProductSP)
    (isDelivered ?product - ProductSP)

    (TimingOn)
    (Running)
    (Accomplished ?product - Product)

    (TimeCheck)
    
  )

  (:functions
  (WorkProgress ?panda_robot - PandaRobot ?product - Product) ; in percentage
  ;distance
  (Distance ?from - Location ?to - Location)
  ;mobile_robot velocity
  (Velocity ?mobile_robot - MobileRobot)
  ;consume time
  (ConsumeTime ?productSP - Product)
  (Time)
)
  ; ; ! THIS
  (:durative-action TimeStart
    :parameters ()
    :duration (>= ?duration 0)
    :condition (and (at start (TimingOn))
    ; (at end (Accomplished))
    )
    :effect (and 
              (at start (Running))
              (increase (Time) (* #t 1))
              (at end (not (Running)))
            )
  )

  ; (:durative-action TimeCheck
  ;   :parameters ()
  ;   :duration (= ?duration 0.001)
  ;   :condition (and (at start (>= (Time) 100)))
  ;   :effect (and 
  ;             (at end (TimeCheck))
  ;           )
  ; )


  ; (:durative-action TimeRecord
  ;   :parameters (?product - ProductSP)
  ;   :duration (= ?duration 0.001)
  ;   :condition (and (at start (isAssembled ?product)))
  ;   :effect (and 
  ;             (at start (assign (ConsumeTime ?product) Time))
  ;             (at start (Accomplished))
  ;           )
  ; )

  (:durative-action TimeRecord
    :parameters (?product - Product ?robot - ScrewingRobot)
    :duration (= ?duration 0.001)
    :condition (and (at start (isProductProcessed ?product ?robot)))
    :effect (and 
              (at start (assign (ConsumeTime ?product) Time))
              (at start (Accomplished ?product))
            )
  )

  

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * INSERTION
  (:durative-action InsertionStartSP
    :parameters (?mobile_robot - MobileRobot ?panda_robot - InsertionRobot ?station - Location ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and   (over all (Running))

                       (at start(and 
                      (isRobotatLocation ?mobile_robot ?station)
                      (isProductatRobot ?product ?mobile_robot)
                      (isRobotatLocation ?panda_robot ?station)
                      (isRobotIdle ?mobile_robot)
                      (hasRobotRecovered ?panda_robot)
                  ))
                )
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
                    (over all (Running))

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
  :condition (and   (over all (Running))
                  (at start(and
                  (>= (WorkProgress ?panda_robot ?product) 20)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?mobile_robot ?location)
                ))
  )
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
                    (over all (Running))

               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )

  (:durative-action InsertionStart
    :parameters (?panda_robot - InsertionRobot ?station - Location ?product - Cylinder)
    :duration (= ?duration 0.001)
    :condition (and (at start(and 
                  (isProductatLocation ?product ?station)
                  (isRobotatLocation ?panda_robot ?station)
                  (hasRobotRecovered ?panda_robot)
               ))
                (over all (Running)))

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
                  (over all (Running))

                )
  :effect (and
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%
            (at end (isPerforming ?panda_robot ?product))
           )
  )

  (:durative-action InsertionFinish
  :parameters (?panda_robot - InsertionRobot ?product - Cylinder ?location - Location)
  :duration (= ?duration 0.001)
  :condition (and(at start(and
                  (>= (WorkProgress ?panda_robot ?product) 10)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatLocation ?product ?location)
                ))
                (over all (Running))
              )
  :effect (at start(and
            (assign (WorkProgress ?panda_robot ?product) 0)
            (isProductProcessed ?product ?panda_robot)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
           ))
  )

  ; just recover the pandarobot from the last task
  (:durative-action InsertionRecover
    :parameters (?panda_robot - InsertionRobot ?product - Cylinder)
    :duration (= ?duration 0.001)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
                  (over all (Running))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )
  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * SCREW
  (:durative-action ScrewingStartSP
    :parameters (?mobile_robot - MobileRobot ?panda_robot - ScrewingRobot ?station - Location ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and(at start(and 
                  (isRobotatLocation ?mobile_robot ?station)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?panda_robot ?station)
                  (isRobotIdle ?mobile_robot)
                  (hasRobotRecovered ?panda_robot)
               ))
                (over all (Running)))
    :effect (and 
              (at start (isPerforming ?panda_robot ?product))
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasRobotRecovered ?panda_robot)))
            )
  )

  (:durative-action ScrewingSP
  :parameters (?panda_robot - ScrewingRobot ?product - ProductSP)
  :duration (and (>= ?duration 0) (<= ?duration 1000))
  :condition (and
                  ; (at start (hasRobotRecovered ?panda_robot))
                  (at start (isPerforming ?panda_robot ?product))
                  (at start (<= (WorkProgress ?panda_robot ?product) 1000))
                  (over all (Running))
                )
  :effect (and
            ; (at start (not (hasRobotRecovered ?panda_robot))) 
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%

            (at end (isPerforming ?panda_robot ?product))
            ; (at end(increase (WorkProgress ?panda_robot) 10))
           )
  )

  (:durative-action ScrewingFinishSP
  :parameters (?mobile_robot - MobileRobot ?panda_robot - ScrewingRobot ?product - ProductSP ?location - Location)
  :duration (= ?duration 0.001)
  :condition (and(at start(and
                  (>= (WorkProgress ?panda_robot ?product) 500)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?mobile_robot ?location)
                ))
                (over all (Running)))
  :effect (at end (and

            (assign (WorkProgress ?panda_robot ?product) 0)
            (isProductProcessed ?product ?panda_robot)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
            (isRobotIdle ?mobile_robot)
           ))
  )

  ; just recover the pandarobot from the last task
  (:durative-action ScrewingRecoverSP
    :parameters (?panda_robot - ScrewingRobot ?product - ProductSP)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
                  (over all (Running))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )

  (:durative-action ScrewingStart
    :parameters (?panda_robot - ScrewingRobot ?station - Location ?product - Bolt)
    :duration (= ?duration 0.001)
    :condition (and(at start(and 
                  (isProductatLocation ?product ?station)
                  (isRobotatLocation ?panda_robot ?station)
                  (hasRobotRecovered ?panda_robot)
               ))
                (over all (Running)))
    :effect (and 
                  (at start (not (hasRobotRecovered ?panda_robot)))
                  (at end (isPerforming ?panda_robot ?product))
            )
  )

  (:durative-action Screwing
  :parameters (?panda_robot - ScrewingRobot ?product - Bolt)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (and
                  (at start (isPerforming ?panda_robot ?product))
                  (at start (<= (WorkProgress ?panda_robot ?product) 100))
                  (over all (Running))
                )
  :effect (and
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%
            (at end (isPerforming ?panda_robot ?product))
           )
  )

  (:durative-action ScrewingFinish
  :parameters (?panda_robot - ScrewingRobot ?product - Bolt ?location - Location)
  :duration (= ?duration 0.001)
  :condition (and (at start(and
                  (>= (WorkProgress ?panda_robot ?product) 25)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatLocation ?product ?location)
                ))
                (over all (Running)))
  :effect (at start(and
            (assign (WorkProgress ?panda_robot ?product) 0)
            (isProductProcessed ?product ?panda_robot)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
           ))
  )

  ; just recover the pandarobot from the last task
  (:durative-action ScrewingRecover
    :parameters (?panda_robot - ScrewingRobot ?product - Bolt)
    :duration (= ?duration 0.001)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
                  (over all (Running))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )
  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * inspect
  (:durative-action InspectionApprove
    :parameters (?insertion_robot - InsertionRobot ?screwing_robot - ScrewingRobot ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and(at start(and 
                  (isProductProcessed ?product ?insertion_robot)
                  (isProductProcessed ?product ?screwing_robot)

               ))
                (over all (Running)))
    :effect (and 
              (at end (isAssembled ?product))
            )
  )


  (:durative-action InspectionStartSP
    :parameters (?mobile_robot - MobileRobot ?panda_robot - InspectionRobot ?station - Location ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and(at start(and 
                  (isAssembled ?product)
                  (isRobotatLocation ?mobile_robot ?station)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?panda_robot ?station)
                  (isRobotIdle ?mobile_robot)
                  (hasRobotRecovered ?panda_robot)
               ))
                (over all (Running)))
    :effect (and 
              (at start (isPerforming ?panda_robot ?product))
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasRobotRecovered ?panda_robot)))
            )
  )

  (:durative-action InspectingSP
  :parameters (?panda_robot - InspectionRobot ?product - ProductSP)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (and
                  ; (at start (hasRobotRecovered ?panda_robot))
                  (at start (isPerforming ?panda_robot ?product))
                  (at start (<= (WorkProgress ?panda_robot ?product) 100))
                  (over all (Running))
                )
  :effect (and
            ; (at start (not (hasRobotRecovered ?panda_robot))) 
            (at start (not (isPerforming ?panda_robot ?product)))
            (increase (WorkProgress ?panda_robot ?product) (* #t 1)) ; increase progress by 10%

            (at end (isPerforming ?panda_robot ?product))
            ; (at end(increase (WorkProgress ?panda_robot) 10))
           )
  )

  (:durative-action InspectionFinishSP
  :parameters (?mobile_robot - MobileRobot ?panda_robot - InspectionRobot ?product - ProductSP ?location - Location)
  :duration (= ?duration 0.001)
  :condition (and(at start(and
                  (>= (WorkProgress ?panda_robot ?product) 5)
                  (isRobotatLocation ?panda_robot ?location)
                  (isPerforming ?panda_robot ?product)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotatLocation ?mobile_robot ?location)
                ))
                  (over all (Running)))
  :effect (at end (and
            (isInspected ?product)
            (assign (WorkProgress ?panda_robot ?product) 0)
            (not (isPerforming ?panda_robot ?product))
            (hasFinished ?panda_robot ?product)
            (isRobotIdle ?mobile_robot)
           ))
  )

  ; just recover the pandarobot from the last task
  (:durative-action InspectionRecoverSP
    :parameters (?panda_robot - InspectionRobot ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and 
                  (at start (hasFinished ?panda_robot ?product))
                  (over all (Running))
               )
    :effect (and
              (at start (not (hasFinished ?panda_robot ?product)))
              (at end (hasRobotRecovered ?panda_robot))
            )
  )


  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

  ; ! DISCARDED NO DISTANCE
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
                  (over all (Running))
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
    :duration (= ?duration 3)
    :condition (and 
                  (at start (isProductatLocation ?product ?location))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isRobotIdle ?mobile_robot))
                  (at start (isRobotFree ?mobile_robot))
                  (over all (Running))
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
    :duration (= ?duration 3)
    :condition (and 
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isLocationFree ?location))
                  (at start (isRobotIdle ?mobile_robot))
                  (over all (Running))
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


