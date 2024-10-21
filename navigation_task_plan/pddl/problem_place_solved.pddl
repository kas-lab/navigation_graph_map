(define (problem place)
  (:domain place )

  (:objects
    room_initial room1 room2 room3 room4 roomf - room
    cup - item
  )

  (:init
    (robot_at room_initial)
    (robot_holds cup)
    (item_delivery_location cup room4)

    (path room_initial room1)
    (path room1 room2)
    (path room2 room3)
    (path room3 room4)

    (path roomf room1)
    (path roomf room2)
    (path roomf room3)
    (path roomf room4)
  )

  (:goal
    (item_at_delivery_location cup)
  )
)
