(define (problem assembly_problem)
  (:domain assembly_domain)
  
  (:objects 
    Supermarket_cell - Location
    Delivery_point - Location
      
    Insertion_station_1 - InsertionStation
    Meshing_station_1 - MeshingStation
    Screwing_station_1 - ScrewingStation
    Quality_inspection_station_1 - QualityInspectionStation
      
    Darko - SupermarketRobot
    MiR600 - MobileRobot
    Panda_robot_1 Panda_robot_2 Panda_robot_3 - PandaRobot

    Product_1 - Product
  )

  (:init
    
    (isRobotatLocation MiR600 Supermarket_cell)

    (isFree MiR600)

    (hasFinished Panda_robot_1)
    (hasFinished Panda_robot_2)
    (hasFinished Panda_robot_3)

    (isIdle Supermarket_cell)
    (isIdle Delivery_point)

    (not (hasAssembled Product_1 Insertion_station_1))
    (not (hasAssembled Product_1 Meshing_station_1))
    (not (hasAssembled Product_1 Screwing_station_1))
    (not (hasInspected Product_1))

    (isRobotatLocation Panda_robot_1 Insertion_station_1)
    (isRobotatLocation Panda_robot_2 Meshing_station_1)
    (isRobotatLocation Panda_robot_3 Screwing_station_1)
    (isProductatLocation Product_1 Supermarket_cell)

  )

  (:goal (and
          (hasAssembled Product_1 Insertion_station_1)
          (hasAssembled Product_1 Meshing_station_1)
          (hasAssembled Product_1 Screwing_station_1)
          (hasInspected Product_1)
          (isProductatLocation Product_1 Delivery_point)
         )
  )
)
