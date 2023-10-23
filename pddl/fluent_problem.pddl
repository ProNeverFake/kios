(define (problem fluent_problem)
  (:domain fluent_domain)
  
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
    (isRobotIdle MiR600)
    (isRobotatLocation MiR600 Insertion_station_1)

    ; Insertion_robot
    (hasFinished Insertion_robot)
    (isRobotatLocation Insertion_robot Insertion_station_1)
    (= (WorkProgress Insertion_robot) 44)

    ; product ppt
    (isProductatRobot Product_1 MiR600)
    
    ; location ppt

  )

  (:goal (and
            (isProductAssembled Product_1 Insertion_station_1) 
            (isRobotatLocation MiR600 Delivery_point)
         )
  )
)
