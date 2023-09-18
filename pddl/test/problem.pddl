(define (problem sim_problem)
  (:domain sim_domain)
  
  (:objects 
    supermarket - Supermarket
    
    insertion_station - InsertionStation
    gear_meshing_station - GearMeshingStation
    gear_insertion_station - GearInsertionStation
    screwing_station - ScrewingStation
    inspection_station - InspectionStation

    delivery_point - DeliveryPOint

    productSP1 - ProductSP
    cylinder1 cylinder2 cylinder3 cylinder4 cylinder5 cylinder6 cylinder7 - Cylinder
    bolt1 bolt2 bolt3 bolt4 bolt5 bolt6 bolt7 - Bolt

    MiR600 - MobileRobot

  )

  (:init
    ; timing
    ; (Running)
    (TimingOn)
    (= (ConsumeTime productSP1) 0)
    (= (TIme) 0)

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
    (isRobotatLocation MiR600 supermarket)
    (= (Velocity MiR600) 1)


    ; insertion_robot
    (hasLocationRecovered insertion_station)
    ; (hasFinished insertion_robot cylinder1)

    ; screwing robot
    (hasLocationRecovered screwing_station)

    ; inspection robot
    (hasLocationRecovered inspection_station)

    ; product ppt
    (isProductatLocation productSP1 supermarket)

    (isProductatLocation cylinder1 insertion_station)
    (isProductatLocation bolt1 screwing_station)


    ; location ppt
    (isLocationFree delivery_point)

  )

  (:goal (and
            (hasLocationRecovered insertion_station)
            (hasLocationRecovered screwing_station)
            (isAssembled productSP1)
            ; (isInspected productSP1)

            (isProductatLocation productSP1 delivery_point) 
            (isRobotatLocation MiR600 delivery_point)

            (isProductProcessed cylinder1 insertion_station)

            (isProductProcessed bolt1 screwing_station)

            (Accomplished)
         )
  )

  ; (:metric minimize (ConsumeTime productSP1))
)
