(define (domain place)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
    :durative-actions
  )

  (:types
    room
    item
    action
  )

  (:constants
    move - action
  )

  (:predicates
    (robot_at ?room - room)
    (robot_holds ?item - item)
    (item_at ?item - item ?room - room)
    (item_delivery_location ?item - item ?room - room)
    (item_at_delivery_location ?item - item)
    (path ?room1 ?room2 - room)

    (action_feasible ?a - action)
  )

  (:durative-action move
    :parameters (?room1 ?room2 - room)
    :duration ( = ?duration 5)
    :condition (and
      (at start (robot_at ?room1))
      (over all (path ?room1 ?room2))
    )
    :effect (and
      (at start (not(robot_at ?room1)))
      (at end (robot_at ?room2))
    )
  )

  (:durative-action place
    :parameters (?item - item ?room - room)
    :duration ( = ?duration 5)
    :condition (and
      (at start (robot_at ?room))
      (at start (robot_holds ?item))
      (at start (item_delivery_location ?item ?room))
    )
    :effect (and
      (at end (item_at ?item ?room))
      (at end (item_at_delivery_location ?item))
    )
  )

)
