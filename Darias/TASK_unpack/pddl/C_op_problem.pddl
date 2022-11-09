;15:13:37 08/11

;Skeleton_SN = 4

(define (problem put-wuti_propo_reorder)
   (:domain pick-and-place_propo_reorder)

   (:objects
          q224 - config
          o6 o7 o8 o9 - wuti
          p232 p688 - pose
          _p0 _p1 _p2 _p3 _p4 _p5 _p6 _p7 _p8 _p9 - propo_action
          _s1 _s11 _s12 _s149 _s17 _s18 _s198 _s2 _s20 _s21 _s38 _s4 _s5 _s64 _s72 - propo_stream
   )

   (:init
          (allowlocate)
          (atconf q224)
          (atpose o8 p688)
          (atpose o9 p232)
          (canmove)
          (canpick)
          (fixed o6)
          (fixed o7)
          (graspable o8)
          (graspable o9)
          (handempty)
          (isconf q224)
          (ispose o8 p688)
          (ispose o9 p232)
          (stackable o8 o6)
          (stackable o8 o7)
          (stackable o9 o6)
          (stackable o9 o7)
          (_applicable _p0)
   )

   (:goal
        (_applicable _p9)
   )

)
