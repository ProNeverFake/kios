(define (problem fluent2_problem)
  (:domain fluent2_domain)
  
  (:objects 
    supermarket - Location
    
    insertion_station - Location
    gear_meshing_station - Location
    gear_insertion_station - Location
    screwing_station - Location
    inspection_station - Location

    delivery_point - Location

    productSP1 - ProductSP
    cylinder1 cylinder2 cylinder3 cylinder4 cylinder5 cylinder6 cylinder7 - Cylinder
    bolt1 bolt2 bolt3 bolt4 bolt5 bolt6 bolt7 - Bolt

    MiR600 - MobileRobot
    insertion_robot - InsertionRobot
    screwing_robot - ScrewingRobot
    inspection_robot - InspectionRobot
  )

  (:init
    ; Distance
    (= (Distance supermarket insertion_station) 10)
    (= (Distance supermarket gear_meshing_station) 15)
    (= (Distance supermarket gear_insertion_station) 20)
    (= (Distance supermarket screwing_station) 25)
    (= (Distance supermarket inspection_station) 30)
    (= (Distance supermarket delivery_point) 10)

    (= (Distance insertion_station gear_meshing_station) 5)
    (= (Distance insertion_station gear_insertion_station) 10)
    (= (Distance insertion_station screwing_station) 15)
    (= (Distance insertion_station inspection_station) 20)
    (= (Distance insertion_station delivery_point) 15)

    (= (Distance gear_meshing_station gear_insertion_station) 5)
    (= (Distance gear_meshing_station screwing_station) 10)
    (= (Distance gear_meshing_station inspection_station) 15)
    (= (Distance gear_meshing_station delivery_point) 20)

    (= (Distance gear_insertion_station screwing_station) 5)
    (= (Distance gear_insertion_station inspection_station) 10)
    (= (Distance gear_insertion_station delivery_point) 15)

    (= (Distance screwing_station inspection_station) 5)
    (= (Distance screwing_station delivery_point) 10)

    (= (Distance inspection_station delivery_point) 5)

    ; reverse Distance
    (= (Distance insertion_station supermarket) 10)
    (= (Distance gear_meshing_station supermarket) 15)
    (= (Distance gear_insertion_station supermarket) 20)
    (= (Distance screwing_station supermarket) 25)
    (= (Distance inspection_station supermarket) 30)
    (= (Distance delivery_point supermarket) 10)

    (= (Distance gear_meshing_station insertion_station) 5)
    (= (Distance gear_insertion_station insertion_station) 10)
    (= (Distance screwing_station insertion_station) 15)
    (= (Distance inspection_station insertion_station) 20)
    (= (Distance delivery_point insertion_station) 15)

    (= (Distance gear_insertion_station gear_meshing_station) 5)
    (= (Distance screwing_station gear_meshing_station) 10)
    (= (Distance inspection_station gear_meshing_station) 15)
    (= (Distance delivery_point gear_meshing_station) 20)

    (= (Distance screwing_station gear_insertion_station) 5)
    (= (Distance inspection_station gear_insertion_station) 10)
    (= (Distance delivery_point gear_insertion_station) 15)

    (= (Distance inspection_station screwing_station) 5)
    (= (Distance delivery_point screwing_station) 10)

    (= (Distance delivery_point inspection_station) 5)


    ; MiR600
    (isRobotIdle MiR600)
    (isRobotFree MiR600)
    (isRobotatLocation MiR600 insertion_station)
    (= (Velocity MiR600) 1)


    ; insertion_robot
    (hasRobotRecovered insertion_robot)
    ; (hasFinished insertion_robot cylinder1)
    (isRobotatLocation insertion_robot insertion_station)
    ; (= (WorkProgress insertion_robot productSP1) 0)

    ; screwing robot
    (hasRobotRecovered screwing_robot)
    (isRobotatLocation screwing_robot screwing_station)

    ; inspection robot
    (hasRobotRecovered inspection_robot)
    (isRobotatLocation inspection_robot inspection_station)




    ; product ppt
    ; (isProductatRobot productSP1 MiR600) ; ! CHANGED
    (isProductatLocation productSP1 supermarket)

    (isProductatLocation cylinder1 insertion_station)
    (isProductatLocation cylinder2 insertion_station)
    (isProductatLocation cylinder3 insertion_station)
    (isProductatLocation cylinder4 insertion_station)
    (isProductatLocation cylinder5 insertion_station)
    (isProductatLocation cylinder6 insertion_station)
    (isProductatLocation cylinder7 insertion_station)

    (isProductatLocation bolt1 screwing_station)
    (isProductatLocation bolt2 screwing_station)
    (isProductatLocation bolt3 screwing_station)
    (isProductatLocation bolt4 screwing_station)
    (isProductatLocation bolt5 screwing_station)
    (isProductatLocation bolt6 screwing_station)
    (isProductatLocation bolt7 screwing_station)

    ; location ppt
    (isLocationFree delivery_point)

  )

  (:goal (and
            (hasRobotRecovered insertion_robot)
            (hasRobotRecovered screwing_robot)
            ; (isProductProcessed productSP1 insertion_robot)
            ; (isProductProcessed productSP1 screwing_robot)
            (isAssembled productSP1)
            (isInspected productSP1)

            (isProductatLocation productSP1 delivery_point) 
            (isRobotatLocation MiR600 supermarket)

            (isProductProcessed cylinder1 insertion_robot)
            (isProductProcessed cylinder2 insertion_robot)
            (isProductProcessed cylinder3 insertion_robot)
            (isProductProcessed cylinder4 insertion_robot)
            (isProductProcessed cylinder5 insertion_robot)
            (isProductProcessed cylinder6 insertion_robot)
            (isProductProcessed cylinder7 insertion_robot)

            (isProductProcessed bolt1 screwing_robot)
            (isProductProcessed bolt2 screwing_robot)
            (isProductProcessed bolt3 screwing_robot)
            (isProductProcessed bolt4 screwing_robot)
            (isProductProcessed bolt5 screwing_robot)
            (isProductProcessed bolt6 screwing_robot)
            (isProductProcessed bolt7 screwing_robot)


            ; (forall (?cylinder - Cylinder)
            ;         (isProductProcessed ?cylinder insertion_robot)
            ; )
         )
  )
  ; (:metric minimize (WorkProgress insertion_robot productSP1))
)
