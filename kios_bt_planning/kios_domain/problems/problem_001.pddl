(define (problem robot_assembly_problem-problem)
 (:domain robot_assembly_problem-domain)
 (:objects
   parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool
   gear1 gear2 gear3 shaft1 shaft2 shaft3 gearbase gearbase_hole1 gearbase_hole3 - part
   left_hand - hand
 )
 (:init (can_manipulate parallel_box2 gear1) (can_manipulate outward_claw gear2) (can_manipulate outward_claw gear3) (can_manipulate no_tool shaft3) (can_manipulate parallel_box1 shaft1) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub lampbase) (can_place_to lamp blub) (can_insert_to shaft1 gearbase_hole1) (can_insert_to shaft3 gearbase_hole3) (can_insert_to gear3 shaft3) (can_insert_to gear2 shaft2) (can_insert_to gear1 shaft1))
 (:goal (and ))
)
