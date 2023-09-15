(define (problem test1_problem)
  (:domain test1_domain)
  
  (:objects 
    apple1 - apple
  )

  (:init
    (isapple apple1 )
    (isfruit apple1 )

  )

  (:goal (and
            (testpredicate)
         )
  )
)
