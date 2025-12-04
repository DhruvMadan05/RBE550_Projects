(define (domain blocksworld)
  (:requirements :strips)
  (:predicates
    (on ?x ?y)
    (ontable ?x)
    (clear ?x)
    (holding ?x)
    (handempty)
    (nextTo ?x ?y)
    (nextToCenter ?x ?y ?z)
  )

  (:action pick-up
    :parameters (?x)
    :precondition (and
      (clear ?x)
      (ontable ?x)
      (handempty)
    )
    :effect (and
      (holding ?x)
      (not (ontable ?x))
      (not (clear ?x))
      (not (handempty))
    )
  )

  (:action put-down
    :parameters (?x)
    :precondition (and
      (holding ?x)
    )
    :effect (and
      (ontable ?x)
      (clear ?x)
      (handempty)
      (not (holding ?x))
    )
  )

  (:action stack
    :parameters (?x ?y)
    :precondition (and
      (holding ?x)
      (clear ?y)
    )
    :effect (and
      (on ?x ?y)
      (clear ?x)
      (handempty)
      (not (holding ?x))
      (not (clear ?y))
    )
  )

  (:action unstack
    :parameters (?x ?y)
    :precondition (and
      (on ?x ?y)
      (clear ?x)
      (handempty)
    )
    :effect (and
      (holding ?x)
      (clear ?y)
      (not (on ?x ?y))
      (not (clear ?x))
      (not (handempty))
    )
  )

  (:action place-next-to
    :parameters (?x ?y)
    :precondition (and
      (holding ?x)
      (ontable ?y)
    )
    :effect (and
      (ontable ?x)
      (clear ?x)
      (handempty)
      (nextTo ?x ?y)
      (nextTo ?y ?x)
      (not (holding ?x))
    )
  )

  (:action place-next-to-center
    :parameters (?x ?y ?z)
    :precondition (and
      (holding ?x)
      (ontable ?y)
      (ontable ?z)
      (nextTo ?y ?z)
    )
    :effect (and
      (ontable ?x)
      (clear ?x)
      (handempty)
      (nextToCenter ?x ?y ?z)
      (nextToCenter ?x ?z ?y)
      (not (holding ?x))
    )
  )
)
