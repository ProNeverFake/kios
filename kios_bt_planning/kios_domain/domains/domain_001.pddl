(define (domain robot_assembly_problem-domain)
 (:requirements :strips :typing)
 (:types
    thing - object
    tool part hand - thing
 )
 (:predicates (is_free ?p - thing) (is_equippable ?tool - tool) (can_manipulate ?tool - tool ?part - part) (can_insert_to ?part1 - part ?part2 - part) (can_screw_to ?part1 - part ?part2 - part) (can_place_to ?part1 - part ?part2 - part) (hold ?thing1 - thing ?thing2 - thing) (is_inserted_to ?part1 - part ?part2 - part) (is_screwed_to ?part1 - part ?part2 - part) (is_placed_to ?part1 - part ?part2 - part))
 (:action pick_up
  :parameters ( ?hand - hand ?tool - tool ?part - part)
  :precondition (and (is_free ?tool) (hold ?hand ?tool) (can_manipulate ?tool ?part))
  :effect (and (hold ?tool ?part) (not (is_free ?tool))))
 (:action put_down
  :parameters ( ?hand - hand ?tool - tool ?part - part)
  :precondition (and (hold ?tool ?part) (hold ?hand ?tool))
  :effect (and (not (hold ?hand ?part)) (is_free ?tool)))
 (:action place
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (hold ?tool ?part1) (can_place_to ?part1 ?part2))
  :effect (and (not (hold ?tool ?part1)) (is_free ?tool) (is_placed_to ?part1 ?part2)))
 (:action detach
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (is_free ?tool) (can_manipulate ?tool ?part1) (is_placed_to ?part1 ?part2))
  :effect (and (hold ?tool ?part1) (not (is_free ?tool)) (not (is_placed_to ?part1 ?part2))))
 (:action insert
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (hold ?tool ?part1) (can_insert_to ?part1 ?part2))
  :effect (and (not (hold ?tool ?part1)) (is_free ?tool) (is_inserted_to ?part1 ?part2)))
 (:action pull
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (is_free ?tool) (is_inserted_to ?part1 ?part2) (can_manipulate ?tool ?part1))
  :effect (and (hold ?tool ?part1) (not (is_free ?tool)) (not (is_inserted_to ?part1 ?part2))))
 (:action screw
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (hold ?tool ?part1) (can_screw_to ?part1 ?part2))
  :effect (and (not (hold ?tool ?part1)) (is_free ?tool) (is_screwed_to ?part1 ?part2)))
 (:action unscrew
  :parameters ( ?hand - hand ?tool - tool ?part1 - part ?part2 - part)
  :precondition (and (hold ?hand ?tool) (is_free ?tool) (is_screwed_to ?part1 ?part2) (can_manipulate ?tool ?part1))
  :effect (and (hold ?tool ?part1) (not (is_free ?tool)) (not (is_screwed_to ?part1 ?part2))))
 (:action load_tool
  :parameters ( ?hand - hand ?tool - tool)
  :precondition (and (is_equippable ?tool) (is_free ?hand))
  :effect (and (not (is_free ?hand)) (not (is_equippable ?tool)) (hold ?hand ?tool)))
 (:action unload_tool
  :parameters ( ?hand - hand ?tool - tool)
  :precondition (and (hold ?hand ?tool) (is_free ?tool))
  :effect (and (is_free ?hand) (is_equippable ?tool) (not (hold ?hand ?tool))))
)
