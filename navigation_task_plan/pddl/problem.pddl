(define (problem navigation)
  (:domain navigation )

  (:objects
    wp1 wp2 wp3 wp4 wpf - waypoint
  )

  (:init
    (robot_at wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)

    (connected wpf wp1)
    (connected wpf wp2)
    (connected wpf wp3)
    (connected wpf wp4)
  )

  (:goal
    (robot_at wp4)
  )
)
