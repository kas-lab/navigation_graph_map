(define (problem place)
  (:domain place )

  (:objects
    garage - room
    cup - item
  )

  (:init
    (robot_at garage)
    (robot_holds cup)
  )

  (:goal
    (item_at_delivery_location cup)
  )
)
