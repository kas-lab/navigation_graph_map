(define (domain navigation)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
    :durative-actions
  )

  (:types
    waypoint
    action
  )

  (:constants
    move - action
  )

  (:predicates
    (robot_at ?wp - waypoint)
    (connected ?wp1 ?wp2 - waypoint)

    (action_feasible ?a - action)
  )

  (:durative-action move
    :parameters (?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
      (at start (robot_at ?wp1))
      (over all (connected ?wp1 ?wp2))
      (over all (action_feasible move))
    )
    :effect (and
      (at start (not(robot_at ?wp1)))
      (at end (robot_at ?wp2))
    )
  )

)
