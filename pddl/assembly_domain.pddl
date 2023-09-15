
(define (domain assembly_domain)
  (:requirements :strips :typing :adl :fluents :durative-actions)
  
  (:types 
    Product
    InsertionStation MeshingStation ScrewingStation QualityInspectionStation - Location
    MobileRobot PandaRobot SupermarketRobot - Robot
  )

  (:predicates
    (isRobotatLocation ?robot - Robot ?location - Location)
    (isProductatLocation ?product - Product ?location - Location)
    (isProductatRobot ?product - Product ?robot - Robot)

    (hasFinished ?panda_robot - PandaRobot)
    (isFree ?mobile_robot - MobileRobot)
    (isIdle ?location - Location)

    (hasRecovered ?robot - PandaRobot)
    (hasAssembled ?product - Product ?location - Location)
    (hasInspected ?product - Product)
  )

  (:durative-action Move
    :parameters (?mr - MobileRobot ?from - Location ?to - Location)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (isRobotatLocation ?mr ?from))
               )
    :effect (and 
              (at start (not (isRobotatLocation ?mr ?from)))
              (at end (isRobotatLocation ?mr ?to))
            )
  )

  (:durative-action Load
    :parameters (?mobile_robot - MobileRobot ?product - Product ?location - Location)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (isFree ?mobile_robot))
                  (at start (isProductatLocation ?product ?location))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isIdle ?location))
               )
    :effect (and 
              (at start (not (isIdle ?location)))
              (at start (not (isProductatLocation ?product ?location)))
              (at end (isIdle ?location))
              (at end (not (isFree ?mobile_robot)))
              (at end (isProductatRobot ?product ?mobile_robot))
            )
  )

  (:durative-action Unload
    :parameters (?mobile_robot - MobileRobot ?product - Product ?location - Location)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isIdle ?location))
              )
    :effect (and 
              (at start (not (isIdle ?location)))
              (at start (not (isProductatRobot ?product ?mobile_robot)))
              (at end (isProductatLocation ?product ?location))
              (at end (isIdle ?location))
              (at end (isFree ?mobile_robot))
            )
  )


  (:durative-action Insertion
    :parameters (?mobile_robot - MobileRobot ?panda_robot - PandaRobot ?location - InsertionStation ?product - Product)
    :duration (= ?duration 3) ; Different durations for special and regular products
    :condition (and 
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?panda_robot ?location))
                  (at start (hasRecovered ?panda_robot))
               )
    :effect (and 
              (at start (not (hasRecovered ?panda_robot)))
              (at end (hasAssembled ?product ?location))
              (at end (hasFinished ?panda_robot))
            )
  )


  (:durative-action Meshing
    :parameters (?mobile_robot - MobileRobot ?panda_robot - PandaRobot ?location - MeshingStation ?product - Product)
    :duration (= ?duration 5) ; Different durations for special and regular products
    :condition (and 
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?panda_robot ?location))
                  (at start (hasRecovered ?panda_robot))
               )
    :effect (and 
              (at end (hasAssembled ?product ?location))
              (at end (hasFinished ?panda_robot))
              (at start (not (hasRecovered ?panda_robot)))
            )
  )

  (:durative-action Screwing
    :parameters (?mobile_robot - MobileRobot ?panda_robot - PandaRobot ?location - ScrewingStation ?product - Product)
    :duration (= ?duration 7) ; Different durations for special and regular products
    :condition (and 
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (isRobotatLocation ?panda_robot ?location))
                  (at start (hasRecovered ?panda_robot))
               )
    :effect (and 
              (at end (hasAssembled ?product ?location))
              (at end (hasFinished ?panda_robot))
              (at start (not (hasRecovered ?panda_robot)))
            )
  )


  (:durative-action QualityInspection
    :parameters (?mobile_robot - MobileRobot ?location - QualityInspectionStation ?l1 - InsertionStation ?l2 - MeshingStation ?l3 - ScrewingStation ?product - Product)
    :duration (= ?duration 3) ; Different durations for special and regular products
    :condition (and 
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isProductatRobot ?product ?mobile_robot))
                  (at start (hasAssembled ?product ?l1))
                  (at start (hasAssembled ?product ?l2))
                  (at start (hasAssembled ?product ?l3))
               )
    :effect (and 
              (at end (hasInspected ?product))
            )
  )


  (:durative-action Recover
    :parameters (?panda_robot - PandaRobot ?location - InsertionStation)
    :duration (= ?duration 3) ; Different durations for special and regular products
    :condition (and 
                  (at start (isIdle ?location))
                  (at start (hasFinished ?panda_robot))
                  (at start (isRobotatLocation ?panda_robot ?location))
               )
    :effect (and 
              (at start (not (hasFinished ?panda_robot)))
              (at end (hasRecovered ?panda_robot))
            )
  )

)


