; ! only move and insertSP
(define (domain test2_domain)
  (:requirements :strips :typing :fluents :durative-actions :duration-inequalities :continuous-effects)
  
  (:types 
    Product Robot Location - object
    Bolt GearA GearB Cylinder ProductSP - Product
    MobileRobot PandaRobot - Robot
    InspectionRobot ScrewingRobot GearInsertionRobot GearMeshingRobot InsertionRobot - PandaRobot
  )

  (:predicates
    ; location ppt
    (isLocationFree ?location - Location)

    ; robot common properties
    (isRobotFree ?robot - Robot)
    
    (isRobotatLocation ?robot - Robot ?location - Location)
    (isRobotIdle ?robot - Robot)
    ; panda robot ppt
    
    (hasRobotRecovered ?panda_robot - PandaRobot)
    ; *
    (isPerforming ?panda_robot - PandaRobot ?product - Product)
    (hasFinished ?panda_robot - PandaRobot ?product - Product)

    (isInsertRobot ?panda_robot - PandaRobot)
    (isScrewingRobot ?panda_robot - PandaRobot)
    (isGearMeshingRobot ?panda_robot - PandaRobot)
    (isGearInsertionRobot ?panda_robot - PandaRobot)
    (isInspectionRobot ?panda_robot - PandaRobot)

    ; mobile robot ppt
    

    ; product ppt
    (isProductatRobot ?product - Product ?robot - Robot)
    (isProductatLocation ?product - Product ?location - Location)
    (isProductProcessed ?product - Product ?robot - PandaRobot)

    ; ! TEST FLAG
    (FLAG)
    
  )

  (:functions
  

  ;workprogress
    ; ! changed
  (WorkProgress ?panda_robot - PandaRobot ?product - Product) ; in percentage
  ; (RecoverProgress ?panda_robot - PandaRobot)
  ;distance
  (Distance ?from - Location ?to - Location)
  ;mobile_robot velocity
  (Velocity ?mobile_robot - MobileRobot)
  
)
  ; * ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ; * Insert
  ; (:durative-action Insert
  ; :parameters (?mobile_robot - MobileRobot ?robot - InsertionRobot ?location - Location ?product - Product)
  ; :duration (and (>= ?duration 0) (<= ?duration 100))
  ; :condition (and
  ;                 ; asked product state
  ;                 (at start (isRobotatLocation ?mobile_robot ?location))
  ;                 (at start (isProductatRobot ?product ?mobile_robot))
  ;                 ; asked robot state
  ;                 (at start (isRobotIdle ?mobile_robot))
  ;                 (at start (hasRobotRecovered ?robot))
  ;                 ; asked progress
  ;                 (at start (< (WorkProgress ?robot) 100))
  ;               )
  ; :effect (and
  ;           (at start (not (isRobotIdle ?mobile_robot)))
  ;           (at start (isPerforming ?robot))
  ;           (at start (not (hasRobotRecovered ?robot)))
  ;           (increase (WorkProgress ?robot) (* #t 1))
  ;          )
  ; )

  ; (:durative-action InsertFinish
  ; :parameters (?mobile_robot - Robot ?panda_robot - PandaRobot ?product - Product ?location - Location)
  ; :duration (= ?duration 0)
  ; :condition (and
  ;                 (at start (>= (WorkProgress ?panda_robot) 100))
  ;                 (at start (isRobotatLocation ?panda_robot ?location))
  ;                 (at start (isPerforming ?panda_robot))
  ;                 (at start (isProductatRobot ?product ?mobile_robot))
  ;                 (at start (isRobotatLocation ?mobile_robot ?location))
  ;               )
  ; :effect (and
  ;           (at end (assign (WorkProgress ?panda_robot) 0))
  ;           (at end (isProductAssembled ?product ?Location))
  ;           (at end (not (isPerforming ?panda_robot)))
  ;           (at end (hasFinished ?panda_robot))
  ;           (at end (isRobotIdle ?mobile_robot))
  ;          )
  ; )

  ; (:durative-action InsertionRecover
  ;   :parameters (?robot - InsertionRobot ?location - Location)
  ;   :duration (= ?duration 3)
  ;   :condition (and 
  ;                 (at start (hasFinished ?robot))
  ;              )
  ;   :effect (and
  ;             (at start (not (hasFinished ?robot)))
  ;             (at end (hasRobotRecovered ?robot))
  ;           )
  ; )
 

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:durative-action InsertStartSP
  :parameters (?mobile_robot - MobileRobot ?robot - InsertionRobot ?product - ProductSP ?location - Location)
  :duration (= ?duration 0.001)
  :condition (at start (and 
                  (hasRobotRecovered ?robot)
                  ; for SP
                  (isRobotatLocation ?mobile_robot ?location)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotIdle ?mobile_robot)

                  ; asked robot state
                  (isRobotatLocation ?robot ?location)
                  )
                )
  :effect (and
            ; state flow
            
            (at start (not (hasRobotRecovered ?robot))); to disable overlap conductions
            ; lock mobile robot.
            (at start (not (isRobotIdle ?mobile_robot)))
            (at end (isPerforming ?robot ?product))
            ; (increase (WorkProgress ?robot) (* #t 1))            
           )
  )

  ; (:durative-action InsertStart
  ; :parameters (?mobile_robot - MobileRobot ?robot - InsertionRobot ?product - Cylinder ?location - Location)
  ; :duration (= ?duration 0.001)
  ; :condition (at start (and 
  ;                 (hasRobotRecovered ?robot)
  ;                 ; for SP
  ;                 (isProductatLocation ?product ?location)
  ;                 (<= (WorkProgress ?robot) 50)
  ;                 ; asked robot state
  ;                 (isRobotatLocation ?robot ?location)
  ;                 )
  ;               )
  ; :effect (and
  ;           ; state flow
  ;           (at start (isPerforming ?robot ?product))
  ;           (at start (not (hasRobotRecovered ?robot))); to disable overlap conductions
  ;           ; lock mobile robot.
  ;           (at start (not (isRobotIdle ?mobile_robot)))
  ;          )
  ; )

  (:durative-action InsertingSP
  :parameters (?robot - InsertionRobot ?product - ProductSP)
  :duration (and (>= ?duration 0) (<= ?duration 100))
  :condition (at start (and 
                  (isPerforming ?robot ?product)
                  (<= (WorkProgress ?robot ?product) 100)
                  (isRobotIdle ?robot)
                  ; asked robot state
                  )
                )
  :effect (and
            (at start (not (isRobotIdle ?robot)))
            (increase (WorkProgress ?robot ?product) (* #t 1))   
            (at end (isRobotIdle ?robot))         
           )
  )

  ; (:durative-action TESTFLAG
  ; :parameters (?robot - InsertionRobot ?product - ProductSP)
  ; :duration (= ?duration 0.01)
  ; :condition (at start (and 
  ;                 (>= (WorkProgress ?robot ?product) 25)

  ;                 ; asked robot state
  ;                 )
  ;               )
  ; :effect (and
  ;           (at end (FLAG))
            
  ;           )       
           
  ; )



  ; (:durative-action Inserting
  ; :parameters (?mobile_robot - MobileRobot ?robot - InsertionRobot ?product - Cylinder ?location - Location)
  ; :duration (and (>= ?duration 0) (<= ?duration 50))
  ; :condition (at start (and 
  ;                 (isPerforming ?robot ?product)
  ;                 (<= (WorkProgress ?robot) 50)
  ;                 ; for SP
  ;                 (isRobotatLocation ?mobile_robot ?location)
  ;                 (isProductatRobot ?product ?mobile_robot)
  ;                 (isRobotIdle ?mobile_robot)

  ;                 ; asked robot state
  ;                 (isRobotatLocation ?robot ?location)
  ;                 )
  ;               )
  ; :effect (and
  ;           ; state flow
  ;           (at start (isPerforming ?robot))
  ;           (at start (not (hasRobotRecovered ?robot))); to disable overlap conductions
  ;           ; lock mobile robot.
  ;           (at start (not (isRobotIdle ?mobile_robot)))

  ;           (increase (WorkProgress ?robot) (* #t 1))            
  ;          )
  ; )
  
  (:durative-action InsertFinishSP
  :parameters (?mobile_robot - MobileRobot ?robot - InsertionRobot ?product - ProductSP ?location - Location)
  :duration (= ?duration 0.01) ; ! CANNOT BE ZERO!!!!!!
  :condition (at start 
                  (and 
                  (>= (WorkProgress ?robot ?product) 30)
                  (isPerforming ?robot ?product)
                  
                  (isRobotatLocation ?robot ?location)
                  ; for SP
                  (isRobotatLocation ?mobile_robot ?location)
                  (isProductatRobot ?product ?mobile_robot)
                  )

                  
                )
  :effect (and
            ; state flow
            (at end (not (isPerforming ?robot ?product)))
            (at end (hasFinished ?robot ?product))
            ; reset robot
            ; (at end (assign (WorkProgress ?robot ?product) 0))
            ; product processed
            (at end (isProductProcessed ?product ?robot))
            ; release mobile robot
            (at end (isRobotIdle ?mobile_robot))
           )
  )


  ; (:durative-action InsertRecover
  ;   :parameters (?robot - InsertionRobot ?product - Cylinder)
  ;   :duration (= ?duration 20)
  ;   :condition (and 
  ;                 ; state flow
  ;                 (at start (hasFinished ?robot ?product))
  ;              )
  ;   :effect (and
  ;             ; state flow
  ;             (at start (not (hasFinished ?robot ?product)))
  ;             (at end (hasRobotRecovered ?robot))
  ;           )
  ; )

  (:durative-action InsertRecoverSP
    :parameters (?robot - InsertionRobot ?product - ProductSP)
    :duration (= ?duration 1)
    :condition (and 
                  ; state flow
                  (at start (hasFinished ?robot ?product))
               )
    :effect (and
              ; state flow
              (at start (not (hasFinished ?robot ?product)))
              (at end (hasRobotRecovered ?robot))
            )
  )
  ; * ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


  ; (:durative-action Recover
  ;   :parameters (?panda_robot - PandaRobot ?station - (either InsertionStation MeshingStation ))
  ;   :duration (= ?duration 2)
  ;   :condition (and 
  ;                 (at start (hasFinished ?panda_robot))
  ;              )
  ;   :effect (and
  ;             (at start (not (hasFinished ?panda_robot)))
  ;             (at end (hasRobotRecovered ?panda_robot))
  ;           )
  ; )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

  ; * TYPE SAFE
  (:durative-action MoveReverse
    :parameters (?mr - MobileRobot ?from - Location ?to - Location)
    :duration (= ?duration (/ (Distance ?to ?from) (Velocity ?mr)))
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
  
  ; * SAFE
  (:durative-action Load
    :parameters (?mobile_robot - MobileRobot ?product - ProductSP ?location - Location)
    :duration (= ?duration 1)
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
    :duration (= ?duration 1)
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


