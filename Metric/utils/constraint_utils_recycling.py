from itertools import combinations
import numpy as np
from z3 import *

from utils.grid_utils import get_neighbors


# def reachable_set_constraints(solver, robotPos, grid, planHorizon, reach):
#     '''Constraints defining the reachable set from every location.'''
#     numBots = len(robotPos[0])
#     numStates = grid.size
# 
#     for k in range(planHorizon-1):
#         # Reachable set. Have to add a set of or statements for
#         # every possible state.
#         for i in range(numBots):
#             for x in range(numStates):
#                 cons = [(robotPos[k+1][i] == int(x_next)) for x_next in reach[x]]
#                 solver.add( Implies(robotPos[k][i] == int(x), Or(cons)) )



def one_robot_visit_station_for_duration(solver: Solver, robotPos: ArithRef,
                station, grid, obstacles, stations, visitStart: int,
                visitDeadline: int, visitDuration: int):
    ''''''
    station_neighbors = get_neighbors(station[0], grid, obstacles, stations)
    spot = station_neighbors[0]
    
    cons = [
        And(
            [robotPos[k+l][0] == int(spot) for l in range(visitDuration)]
        )
        for k in range(visitStart, visitDeadline-visitDuration)
    ]
    solver.add(Or(cons))

def two_robot_visit_station_for_duration(solver: Solver, robotPos: ArithRef,
                station, grid, obstacles, stations, numVisitors,
                visitStart: int, visitDeadline: int, visitDuration: int):
    ''''''
    station_neighbors = get_neighbors(station[0], grid, obstacles, stations)
    spots = [station_neighbors[i] for i in range(numVisitors)]
    # for i in range(len(spots)):
    #     print(f"Desired spot {i}: {spots[i]}")

    cons = [
        And([
            And(
                robotPos[k+l][0] == int(spots[0]),
                robotPos[k+l][1] == int(spots[1]))
            for l in range(visitDuration)
        ])
        for k in range(visitStart, visitDeadline-visitDuration)
    ]
    solver.add(Or(cons))



def one_robot_visit_station_for_duration_occupied(solver: Solver,
                robotPos: ArithRef, occupied: BoolRef, station: list,
                grid: np.ndarray, obstacles: list, stations:list, 
                visitStart: int, visitDeadline: int, visitDuration: int):
    ''''''
    numBots = len(robotPos[0])
    station_neighbors = get_neighbors(station[0], grid, obstacles, stations)
    spot = station_neighbors
    solver.add(
        Or([
            Or([
                And(
                    robotPos[k][i] == int(spot[0]), 
                    Not(occupied[k][i]),
                    And([
                        And(
                            robotPos[k+l][i] == int(spot[0]),
                            occupied[k+l][i]
                        )
                        for l in range(1, visitDuration)
                    ])) for i in range(numBots)
            ]) for k in range(visitStart, visitDeadline-visitDuration)
        ])
    )

def one_robot_visit_station_then_another_occupied(solver: Solver,
                robotPos: ArithRef, occupied: BoolRef, visit1: list,
                visit2: list, grid: np.ndarray, obstacles: list,
                stations:list, visitStart: int, visitDeadline: int,
                visitDuration: int):
    numBots = len(robotPos[0])
    spot1 = get_neighbors(visit1[0], grid, obstacles, stations)
    spot2 = get_neighbors(visit2[0], grid, obstacles, stations)

    solver.add(
        Or([
            Or([
                And(
                    robotPos[k][i] == int(spot1[0]),
                    Not(occupied[k][i]),
                    Or([
                        And(
                            robotPos[k+l][i] == int(spot2[0]),
                            And([
                                occupied[k+m][i] for m in range(1, l)
                            ]),
                            Not(occupied[k+l][i])
                        )
                        for l in range(1, visitDeadline-visitDuration-k)
                    ])
                )
            for i in range(numBots)])
        for k in range(visitStart, visitDeadline-visitDuration)])
    )

def one_robot_visit_station_then_another_occupied_duration(
    solver, grid, obstacles, stations, robotPos, occupied,
    visit1, visit2, visitStart, visitDeadline,
    visit1Duration, visit2Duration
):
    numBots = len(robotPos[0])
    spot1 = get_neighbors(visit1[0], grid, obstacles, stations)
    spot2 = get_neighbors(visit2[0], grid, obstacles, stations)
    solver.add(
        Or([
            Or([
                And(
                    robotPos[k][i] == int(spot1[0]),
                    Not(occupied[k][i]),
                    And([
                        And(
                            robotPos[k+n][i] == int(spot1[0]),
                            occupied[k+n][i]
                        )
                        for n in range(1, visit1Duration)
                    ]),
                    Or([
                        And(
                            robotPos[k+l][i] == int(spot2[0]),
                            And([
                                And(
                                    robotPos[k+l+p][i] == int(spot2[0]),
                                    occupied[k+l+p][i]
                                )
                                for p in range(1, visit2Duration)
                            ]),
                            And([
                                occupied[k+m][i]
                                for m in range(1, l)
                            ]),
                            Not(occupied[k+l+visit2Duration][i])
                        )
                        for l in range(visit1Duration, visitDeadline-visit2Duration-k)
                    ])
                )
                for i in range(numBots)
            ])
            for k in range(visitStart, visitDeadline-visit1Duration)
        ])
    )