;15:13:47 08/11

;Skeleton_SN = 0

(define (problem put-wuti_propo)
   (:domain pick-and-place_propo)

   (:objects
          q880 - config
          o6 o7 o8 o9 - wuti
          p200 p800 - pose
          _p0 _p1 _p2 _p3 _p4 _p5 - propo_action
   )

   (:init
          (allowlocate)
          (atconf q880)
          (atpose o8 p800)
          (atpose o9 p200)
          (canmove)
          (canpick)
          (fixed o6)
          (fixed o7)
          (graspable o8)
          (graspable o9)
          (handempty)
          (isconf q880)
          (ispose o8 p800)
          (ispose o9 p200)
          (stackable o8 o6)
          (stackable o8 o7)
          (stackable o9 o6)
          (stackable o9 o7)
          (_applicable _p0)
   )

   (:goal
        (_applicable _p5)
   )

)
