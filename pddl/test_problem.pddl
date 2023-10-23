(define (problem test_problem)
  (:domain test_domain)
  
  (:objects 
    Supermarket - Location
    Delivery_point - Location
    Insertion_station_1 - Location

    Product_1 - Product

    MiR600 - MobileRobot
    Insertion_robot - PandaRobot
  )

  (:init
    ; MiR600
    (isRobotFree MiR600)
    (isRobotIdle MiR600)
    (isRobotatLocation MiR600 Supermarket)

    ; Insertion_robot
    (isRobotIdle Insertion_robot)
    (isRobotatLocation Insertion_robot Insertion_station_1)

    ; (isRobotatLocation MiR600 Insertion_station_1)
    ; (isProductatRobot Product_1 MiR600)   

    ; (hasRobotRecovered Insertion_robot)
    
    ; product ppt
    (isProductatLocation Product_1 Supermarket)
    
    ; location ppt
    ; (isLocationFree Delivery_point)

  )

  (:goal (and
          ; (isProductAssembled Product_1 Insertion_station_1)
          ; (isProductatLocation Product_1 Delivery_point)
            
            (isProductatLocation Product_1 Delivery_point)
            (isRobotatLocation MiR600 Insertion_station_1) 
            (isProductAssembled Product_1 Insertion_station_1)
         )
  )
)
