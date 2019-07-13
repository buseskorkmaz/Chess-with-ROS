(define (domain chessworld)
  (:requirements :strips)
(:predicates (freecell ?x)
             (on ?x ?y)
	     (arm-empty)
             (holding ?x)
             (deadpiece ?x))

(:action pickup
  :parameters (?ob ?cell)
  :precondition (and (on ?ob ?cell) (arm-empty))
  :effect (and (holding ?ob) (freecell ?cell) (not (on ?ob ?cell))
               (not (arm-empty))))

(:action putdown
  :parameters  (?ob ?cell)
  :precondition (and (holding ?ob) (freecell ?cell))
  :effect (and (arm-empty) (on ?ob ?cell) (not (freecell ?cell))
               (not (holding ?ob))))

(:action throwaway
  :parameters  (?ob)
  :precondition (holding ?ob)
  :effect (and (arm-empty) (not (holding ?ob)) (deadpiece ?ob))
)
)
