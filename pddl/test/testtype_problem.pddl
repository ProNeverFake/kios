(define (problem type_problem)
  (:domain type_domain)
  
  (:objects 
    supermarket - location
    
    insertion_station - InsertionStation
    gear_meshing_station - location
    gear_insertion_station - location
    screwing_station - screwingstation
    inspection_station - location

    delivery_point - location


    MiR600 - mobilerobot
  )

  (:init

    ; MiR600
    (isRobotIdle MiR600)
    (isRobotatLocation MiR600 insertion_station)

    ; location ppt

  )

  (:goal (and
            (isRobotatLocation MiR600 supermarket)
         )
  )
)
