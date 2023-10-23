(define (problem durative_problem)
  (:domain durative_domain)
  
  (:objects 
    supermarket - Location
    
    insertion_station - Location
    gear_meshing_station - Location
    gear_insertion_station - Location
    screwing_station - Location
    inspection_station - Location

    delivery_point - Location

    productSP1 - ProductSP
    cylinder1 cylinder2 cylinder3 - Cylinder

    MiR600 - MobileRobot
    insertion_robot - InsertionRobot
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

    ; product ppt
    ; (isProductatRobot productSP1 MiR600) ; ! CHANGED
    (isProductatLocation productSP1 supermarket)

    (isProductatLocation cylinder1 insertion_station)
    (isProductatLocation cylinder2 insertion_station)
    (isProductatLocation cylinder3 insertion_station)

    ; location ppt
    (isLocationFree delivery_point)

  )

  (:goal (and
            (hasRobotRecovered insertion_robot)
            (isProductProcessed productSP1 insertion_robot)
            (isProductatLocation productSP1 delivery_point) 
            (isRobotatLocation MiR600 supermarket)
            ; (isProductProcessed cylinder1 insertion_robot) 
            ; (isProductProcessed cylinder2 insertion_robot) 
            ; (isProductProcessed cylinder3 insertion_robot) 
         )
  )
)
