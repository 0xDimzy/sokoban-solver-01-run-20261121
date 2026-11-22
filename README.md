# Roomwise — push-optimal Sokoban solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/0xDimzy/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/0xDimzy/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver built for the
[Build the best Sokoban solver](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty on [make.fun](https://make.fun).

The levels are a one-wide hallway with independent rooms off it: a box pushed
through a door into the hallway can never be pushed back, so every box stays
in its room and the level's push optimum is the sum of the rooms' optima.
The solver cuts the hallway's dead squares out of the floor to find the rooms,
then solves each room push-optimally with A* over pushes:

- state = sorted box squares + the player's reachable region (named by its
  lowest square), so walking between pushes is free in the search;
- heuristic = minimum-cost matching of boxes to goals over precomputed
  single-box push distances (admissible: it ignores the other boxes);
- pruning = dead squares (no lone box can be pushed from there to any goal)
  and frozen 2x2 blocks of walls and boxes with a box off its goal.

Work is budgeted by expansions, never by the clock, and every tie breaks by
square order, so two runs write identical bytes. If a level does not decompose
into rooms, the same A* runs on the whole level.

## Run

```sh
node main.ts
```

Reads `/input/<id>/level.txt` (XSB) and writes `/output/<id>/solution.txt`
(LURD moves) for every instance. For a local run, set
`MAKEFUN_INPUT_DIR` and `MAKEFUN_OUTPUT_DIR`.

## License

MIT — see [LICENSE](LICENSE).
