(define (domain test_domain)
  (:requirements :strips :typing :fluents :durative-actions)
  
  (:types 
    Product Robot Location - Object
    
    PandaRobot MobileRobot - Robot
  )

  (:predicates
    ; robot common properties
    (isRobotFree ?robot - Robot)
    (isRobotIdle ?robot - Robot)
    (isRobotatLocation ?robot - Robot ?location - Location)

    ; panda robot ppt
    (hasRobotRecovered ?panda_robot - PandaRobot)

    ; mobile robot ppt
    
    ; product ppt
    (isProductatRobot ?product - Product ?robot - Robot)
    (isProductatLocation ?product - Product ?location - Location)
    (isProductAssembled ?product - Product ?location - Location)
    
    ; location ppt
    (isLocationFree ?location - Location)
  )

  ;*safe
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

  ;*safe
   (:durative-action Load
    :parameters (?mobile_robot - MobileRobot ?product - Product ?location - Location)
    :duration (= ?duration 1)
    :condition (and 
                  (at start (isProductatLocation ?product ?location))
                  (at start (isRobotatLocation ?mobile_robot ?location))
                  (at start (isRobotFree ?mobile_robot))
                  (at start (isRobotIdle ?mobile_robot))
               )
    :effect (and 
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (isProductatLocation ?product ?location)))
              (at end (not (isRobotFree ?mobile_robot)))
              (at end (isLocationFree ?location))
              (at end (isProductatRobot ?product ?mobile_robot))
              (at end (isRobotIdle ?mobile_robot))
            )
  )
   
  ;* safe
  (:durative-action Unload
      :parameters (?mobile_robot - MobileRobot ?product - Product ?location - Location)
      :duration (= ?duration 1)
      :condition (and 
                    (at start (isRobotIdle ?mobile_robot))
                    (at start (isProductatRobot ?product ?mobile_robot))
                    (at start (isRobotatLocation ?mobile_robot ?location))
                    (at start (isLocationFree ?location))
                )
      :effect (and 
                (at start (not (isRobotIdle ?mobile_robot)))
                (at start (not (isLocationFree ?location)))
                (at start (not (isProductatRobot ?product ?mobile_robot)))
                (at end (isProductatLocation ?product ?location))
                (at end (isRobotFree ?mobile_robot))
                (at end (isRobotIdle ?mobile_robot))
              )
  )

  ; ! ERROR DEEMED UNSOLVABLE

  (:durative-action Insert
    :parameters (?mobile_robot - MobileRobot ?panda_robot - PandaRobot ?insertion_station - Location ?product - Product)
    :duration (= ?duration 3)
    :condition (and 
                  (at start (isRobotatLocation ?mobile_robot ?insertion_station))
                  (at start (isRobotatLocation ?panda_robot ?insertion_station))

                  (at start (isProductatRobot ?product ?mobile_robot))                  

                  (at start (isRobotIdle ?panda_robot))
                  (at start (isRobotIdle ?mobile_robot))

                  (at start (hasRobotRecovered ?panda_robot))
               )
    :effect (and 
              (at start (not (isRobotIdle ?panda_robot)))
              (at start (not (isRobotIdle ?mobile_robot)))
              (at start (not (hasRobotRecovered ?panda_robot)))

              (at end (isProductAssembled ?product ?insertion_station))

              (at end (isRobotIdle ?panda_robot))
              (at end (isRobotIdle ?mobile_robot))
            )
  )

  ; (:durative-action Insert
  ;   :parameters (?mobile_robot - MobileRobot ?panda_robot - PandaRobot ?insertion_station - InsertionStation ?product - Product)
  ;   :duration (= ?duration 3)
  ;   :condition (and 
  ;                 (at start (isRobotatLocation ?mobile_robot ?insertion_station))
  ;                 (at start (isProductatRobot ?product ?mobile_robot))
  ;                 (at start (isRobotatLocation ?panda_robot ?insertion_station))
  ;                 (at start (hasRobotRecovered ?panda_robot))
  ;              )
  ;   :effect (and 
  ;             (at end (isProductAssembled ?product ?insertion_station))
  ;             (at end (not (hasRobotRecovered ?panda_robot)))
  ;           )
  ; )

  ; (:durative-action Insert
  ;   :parameters (?panda_robot - PandaRobot ?insertion_station - InsertionStation ?product - Product)
  ;   :duration (= ?duration 3)
  ;   :condition (and 
  ;                 (at start (isProductatLocation ?product ?insertion_station))
  ;                 (at start (isRobotatLocation ?panda_robot ?insertion_station))
  ;                 (at start (isRobotIdle ?panda_robot))
  ;                 (at start (hasRobotRecovered ?panda_robot))
  ;              )
  ;   :effect (and 
  ;             (at start (not (isRobotIdle ?panda_robot)))
  ;             (at start (not (hasRobotRecovered ?panda_robot)))
  ;             (at end (isProductAssembled ?product ?insertion_station))
  ;             (at end (isRobotIdle ?panda_robot))
  ;           )
  ; )


  ;*safe
  (:durative-action Recover
    :parameters (?panda_robot - PandaRobot)
    :duration (= ?duration 2)
    :condition (and 
                  (at start (isRobotIdle ?panda_robot))
               )
    :effect (and 
              (at start (not (isRobotIdle ?panda_robot)))
              (at end (hasRobotRecovered ?panda_robot))
              (at end (isRobotIdle ?panda_robot))
            )
  )

)


