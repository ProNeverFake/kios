(define (problem robot_assembly_problem-problem)
 (:domain robot_assembly_problem-domain)
 (:objects
   parallel_box1 parallel_box2 inward_claw outward_claw no_tool - tool
   gear1 gear2 gear3 shaft1 shaft2 shaft3 gearbase - part
   left_hand - hand
   gearbase_hole1 gearbase_hole3 - feature
 )
 (:init (can_manipulate parallel_box2 gear1) (can_manipulate outward_claw gear2) (can_manipulate outward_claw gear3) (can_manipulate no_tool shaft3) (can_manipulate parallel_box1 shaft1) (can_screw_to leg1 seat) (can_screw_to leg2 seat) (can_insert_to back seat) (can_screw_to nut1 seat) (can_screw_to nut2 seat) (can_screw_to blub lampbase) (can_place_to lamp blub) (can_insert_to shaft1 gearbase_hole1) (can_insert_to shaft3 gearbase_hole3) (can_insert_to gear3 shaft3) (can_insert_to gear2 shaft2) (can_insert_to gear1 shaft1) (is_free left_hand) (is_free parallel_box1) (is_free parallel_box2) (is_free inward_claw) (is_free outward_claw) (is_free no_tool) (is_equippable parallel_box1) (is_equippable parallel_box2) (is_equippable inward_claw) (is_equippable outward_claw) (is_equippable no_tool))
 (:goal (and ))
)
