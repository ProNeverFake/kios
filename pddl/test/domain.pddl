(define (domain sim_domain)
  (:requirements :strips :typing :equality :fluents :durative-actions :duration-inequalities :universal-preconditions)
  
  (:types ;! TYPE ISSUE: DON'T USE SUPERTYPE!
    Location Robot Product - object
    InsertionStation ScrewingStation InspectionStation Supermarket DeliveryPoint GearMeshingStation GearInsertionStation - Location
    MobileRobot - Robot
    ProductSP Cylinder Bolt - Product
  )

  (:predicates
    
    ; location
    (isLocationFree ?location - Location)
    (hasLocationRecovered ?location - Location)
    (toRecover ?location - Location)
    (toSPRecover ?location - Location)

    ; robot common properties
    (isRobotFree ?robot - Robot)
    
    (isRobotatLocation ?robot - Robot ?location - Location)

    ; mobile robot ppt
    (isRobotIdle ?robot - Robot)

    ; product ppt
    (isProductatRobot ?product - Product ?robot - Robot)
    (isProductatLocation ?product - Product ?location - Location)

    (isProductProcessed ?product - Product ?station - Location)
    
    ; product sp ppt
    (isInspected ?product - ProductSP)
    (isAssembled ?product - ProductSP)

    (TimingOn)
    (Running)
    (Accomplished)
    
  )

  (:functions
    (SpecialProcessLength ?location) ; not used now.
    (RegularProcessLength ?location)    ; used for calculate the process length left
  ;distance
  (Distance ?from - Location ?to - Location)
  ;mobile_robot velocity
  (Velocity ?mobile_robot - MobileRobot)
  ;consume time
  (ConsumeTime ?productSP - ProductSP)
  (Time)
)
  ; ; ! THIS
  (:durative-action TimeStart
    :parameters ()
    :duration (>= ?duration 0.001)
    :condition (and (at start (TimingOn)))
    :effect (and 
              (at start (Running))
              (increase (Time) (* #t 1))
              (at end (not (Running)))
            )
  )

  (:durative-action TimeRecord
    :parameters (?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and (at start (isAssembled ?product)))
    :effect (and 
              (at start (assign (ConsumeTime ?product) Time))
              (at start (Accomplished))
            )
  )

  

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * INSERTION
  (:durative-action InsertSP
    :parameters (?mobile_robot - MobileRobot ?station - InsertionStation ?product - ProductSP)
    :duration (= ?duration 1)
    :condition (and   (over all (Running))

                       (at start(and 
                      (isRobotatLocation ?mobile_robot ?station)
                      (isProductatRobot ?product ?mobile_robot)
                      (isRobotIdle ?mobile_robot)
                      (hasLocationRecovered ?station)
                  ))
                )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasLocationRecovered ?station)))
              (at end (isRobotIdle ?mobile_robot))
              (at end (isProductProcessed ?product ?station))
              (at end (toSPRecover ?station))
            )
  )
  

  (:durative-action InsertRecoverSP
    :parameters (?station - InsertionStation)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (toSPRecover ?station))
                    (over all (Running))

               )
    :effect (and
              (at start (not (toSPRecover ?station)))
              (at end (hasLocationRecovered ?station))
            )
  )

  (:durative-action Insert
    :parameters (?station - InsertionStation ?product - Cylinder)
    :duration (= ?duration 1)
    :condition (and (at start(and 
                  (isProductatLocation ?product ?station)
                  (hasLocationRecovered ?station)
               ))
                (over all (Running)))

    :effect (and 
                  (at start (not (hasLocationRecovered ?station)))
                  (at end (isProductProcessed ?product ?station))
                  (at end (toRecover ?station))
            )
  )

  (:durative-action InsertRecover
    :parameters (?station - InsertionStation)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (toRecover ?station))
                  (over all (Running))
               )
    :effect (and
              (at start (not (toRecover ?station)))
              (at end (hasLocationRecovered ?station))
            )
  )
  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * SCREW
  (:durative-action ScrewSP
    :parameters (?mobile_robot - MobileRobot ?station - ScrewingStation ?product - ProductSP)
    :duration (= ?duration 1)
    :condition (and   (over all (Running))

                       (at start(and 
                      (isRobotatLocation ?mobile_robot ?station)
                      (isProductatRobot ?product ?mobile_robot)
                      (isRobotIdle ?mobile_robot)
                      (hasLocationRecovered ?station)
                  ))
                )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasLocationRecovered ?station)))
              (at end (isRobotIdle ?mobile_robot))
              (at end (isProductProcessed ?product ?station))
              (at end (toSPRecover ?station))
            )
  )
  

  (:durative-action ScrewRecoverSP
    :parameters (?station - ScrewingStation)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (toSPRecover ?station))
                    (over all (Running))

               )
    :effect (and
              (at start (not (toSPRecover ?station)))
              (at end (hasLocationRecovered ?station))
            )
  )

  (:durative-action Screw
    :parameters (?station - ScrewingStation ?product - Bolt)
    :duration (= ?duration 1)
    :condition (and (at start(and 
                  (isProductatLocation ?product ?station)
                  (hasLocationRecovered ?station)
               ))
                (over all (Running)))

    :effect (and 
                  (at start (not (hasLocationRecovered ?station)))
                  (at end (isProductProcessed ?product ?station))
                  (at end (toRecover ?station))
            )
  )

  (:durative-action ScrewRecover
    :parameters (?station - ScrewingStation)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (toRecover ?station))
                  (over all (Running))
               )
    :effect (and
              (at start (not (toRecover ?station)))
              (at end (hasLocationRecovered ?station))
            )
  )
  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;START
  ; * inspect
  (:durative-action InspectionApprove
    :parameters (?insertion_station - InsertionStation ?screwing_station - ScrewingStation ?product - ProductSP)
    :duration (= ?duration 0.001)
    :condition (and(at start(and 
                  (isProductProcessed ?product ?insertion_station)
                  (isProductProcessed ?product ?screwing_station)

               ))
                (over all (Running)))
    :effect (and 
              (at end (isAssembled ?product))
            )
  )

  (:durative-action InspectSP
    :parameters (?mobile_robot - MobileRobot ?station - InspectionStation ?product - ProductSP)
    :duration (= ?duration 20)
    :condition (and(at start(and 
                  (isAssembled ?product)
                  (isRobotatLocation ?mobile_robot ?station)
                  (isProductatRobot ?product ?mobile_robot)
                  (isRobotIdle ?mobile_robot)
               ))
                (over all (Running)))
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasLocationRecovered ?station)))
              (at end (isRobotIdle ?mobile_robot))
              (at end (isInspected ?product))
              (at end (toSPRecover ?station))
            )
  )

  (:durative-action InspectRecoverSP
    :parameters (?location - InspectionStation)
    :duration (= ?duration 0.001)
    :condition (and 
                (at start (toSPRecover ?location))
                (over all (Running))
               )
    :effect (and
              (at start (not (toSPRecover ?location)))
              (at end (hasLocationRecovered ?location))
            )
  )

  ;*;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;END

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

   ; * SAFE
  (:durative-action Load
    :parameters (?mobile_robot - MobileRobot ?product - ProductSP ?location - Location)
    :duration (= ?duration 5)
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
              (at start (isLocationFree ?location))
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
                  (over all (Running))
              )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (isProductatRobot ?product ?mobile_robot)))
              (at end (isProductatLocation ?product ?location))
              (at end (not (isLocationFree ?location)))
              (at start (isRobotFree ?mobile_robot))
              (at end (isRobotIdle ?mobile_robot))
            )
  )

)


