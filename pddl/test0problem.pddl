(define (problem test0_problem)
  (:domain test0_domain)
  
  (:objects 
    supermarket - Location
    
    insertion_station - Location
    gear_meshing_station - Location
    gear_insertion_station - Location
    screwing_station - Location

    inspection_station - Location

    delivery_point - Location


    ; robots
    MiR600 - MobileRobot

    screwing_robot - ScrewingRobot
    gear_insertion_robot - GearInsertionRobot
    gear_meshing_robot - GearMeshingRobot
    insertion_robot - InsertionRobot

    inspection_robot - InspectionRobot

    
    ; Products
    bolt1 bolt2 bolt3 bolt4 bolt5 - Bolt
    gearA1 gearA2 gearA3 gearA4 gearA5 - GearA
    gearb1 gearb2 gearb3 gearb4 gearb5 - GearB
    cylinder1 cylinder2 cylinder3 cylinder4 cylinder5 - Cylinder

    productSP1 - ProductSP 
    

  )

  (:init
    ; workprogress
    (= (WorkProgress insertion_robot productSP1) 0)
    ; (= (WorkProgress screwing_robot ) 0)
    ; (= (WorkProgress gear_meshing_robot) 0)
    ; (= (WorkProgress gear_insertion_robot) 0)

    ; (= (WorkProgress inspection_robot) 0)

    ; Distance
    ; (= (Distance supermarket delivery_point) 10)

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

    ; MiR600
    (= (Velocity MiR600) 1)
    (isRobotFree MiR600)
    (isRobotIdle MiR600)
    (isRobotatLocation MiR600 delivery_point)
    ; (isRobotatLocation MiR600 insertion_station) 

    ; (isLocationFree Delivery_point)
  
    ; (isProductatRobot productSP1 MiR600)   

    ; Insertion_robot
    (isRobotatLocation insertion_robot insertion_station)
    (hasRobotRecovered insertion_robot)
    (isRobotIdle insertion_robot)
    ; (hasFinished insertion_robot)

    

    ; product ppt
    (isProductatLocation productSP1 supermarket)

    ; location ppt
    (isLocationFree delivery_point)

  )

  (:goal (and  
            ; (isProductatLocation productSP1 delivery_point)
            ; (isRobotatLocation MiR600 supermarket) 
            (isProductProcessed productSP1 insertion_robot)
            (FLAG)
            ; (isRobotatLocation MiR600 insertion_station) 


         )
  )
)
