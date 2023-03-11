'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for building and adding constraints for the Z3 solver.
'''
from itertools import combinations
import numpy as np
from z3 import *

from utils.grid_utils import get_neighbors


def create_robot_occupied_variables(numRobots: int, planHorizon: int):
    '''Create the position and occupied flag variables for the solver.'''
    xString = [
        [f'x{i}{k}' for i in range(1, numRobots+1)] 
        for k in range(planHorizon)
    ]
    X = [Ints(' '.join(xString[i])) for i in range(len(xString))]

    ocString = [
        [f'oc{i}{k}' for i in range(1, numRobots+1)] 
        for k in range(planHorizon)
    ]
    Oc = [Bools(' '.join(ocString[i])) for i in range(len(ocString))]

    return X, Oc

def terminal_position_constraints(
    solver: Solver, robotPos: ArithRef, 
    initialPositions: list, finalPositions: list
):
    '''Specifies the initial and final positions of the robots.'''
    numBots = len(robotPos[0])

    solver.add([robotPos[0][i] == initialPositions[i] for i in range(numBots)])
    solver.add([robotPos[-1][i] == finalPositions[i] for i in range(numBots)])

def avoid_one_another(solver: Solver, robotPos: ArithRef, planHorizon: int):
    '''Two robots cannot be in the same place at the same time.'''
    numBots = len(robotPos[0])

    solver.add([
        And([
            robotPos[k][i] != robotPos[k][j] 
            for i in range(numBots-1) 
            for j in range(i+1, numBots)
        ])

        for k in range(planHorizon-1)
    ])

def stay_in_workspace_constraints(
    solver: Solver, robotPos: ArithRef, grid: np.ndarray, planHorizon: int
):
    '''Keep the robots in the workspace.'''
    numBots = len(robotPos[0])
    numStates = grid.size

    solver.add([
        And(
            robotPos[k][i] < numStates,
            robotPos[k][i] >= 0 
        )
        for k in range(planHorizon) 
        for i in range(numBots)
    ])

def reachable_set_constraints(
    solver: Solver, robotPos: ArithRef, grid: np.ndarray,
    planHorizon: int, reach: np.ndarray
):
    '''Constraints defining the reachable set from every location.'''
    numBots = len(robotPos[0])
    numStates = grid.size

    solver.add(
        [
        Implies(
            robotPos[k][i] == int(x),
            Or(
                [(robotPos[k+1][i] == int(x_next)) for x_next in reach[x]]
            )
        )
        for k in range(planHorizon-1)
        for i in range(numBots)
        for x in range(numStates)
        ]
    )

def check_enough_agents(numRobots: int, numVisitors: int):
    '''Check whether we have enough robots to complete a task.'''
    # TODO: Station has to have at least as many neighbors as robots (for now).
    if numRobots < numVisitors:
        raise ValueError(f"There are not enough robots ({numRobots}) "
                    f"to complete the task specified for {numVisitors}"
                    f" agents.")

def check_enough_neighbors(station: list, station_neighbors: list,
    numVisitors: int):
    '''Station has to have at least as many neighbors as robots (for now).'''
    if len(station_neighbors) < numVisitors:
        raise ValueError(f"Station at {station} has fewer neighbors "
                    f"{len(station_neighbors)} than the number"
                    f" of visiting robots {numVisitors}.")

def n_robots_visit_station_for_duration(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    grid: np.ndarray, obstacles: list, stations: list,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station: list, visitDuration: int
):
    '''Builds a constraint for n robots to visit a station for a duration.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

    station_neighbors = get_neighbors(station[0], grid, obstacles, stations)
    check_enough_neighbors(station, station_neighbors, numVisitors)

    spots = [station_neighbors[i] for i in range(numVisitors)]

    horizonCons = []
    robotIdx = range(numRobots)

    for k in range(visitStart, visitDeadline-visitDuration):
        robotCombos = []

        combos = combinations(robotIdx, numVisitors)
        for combo in combos:
            robotCombos.append(
                And(
                    And([
                        And(
                            robotPos[k][combo[j]] == int(spots[j]),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([
                        And(
                            robotPos[k+l][combo[j]] == int(spots[j]),
                            occupied[k+l][combo[j]]
                        )
                        for j in range(numVisitors)
                        for l in range(1, visitDuration)
                    ]),
                    And([
                        Not(occupied[k+visitDuration][combo[j]])
                        for j in range(numVisitors)
                    ])
                )
            )
        horizonCons.append(Or(robotCombos))
    solver.add(Or(horizonCons))

def n_robots_sequence_two_visits(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    grid: np.ndarray, obstacles: list, stations: list,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station1: list, station2: list,
    visit1Duration: int, visit2Duration: int
):
    '''Constraint for n robots to sequence two diff duration visits.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

    station_neighbors1 = get_neighbors(station1[0], grid, obstacles, stations)
    station_neighbors2 = get_neighbors(station2[0], grid, obstacles, stations)

    check_enough_neighbors(station1, station_neighbors1, numVisitors)
    check_enough_neighbors(station2, station_neighbors2, numVisitors)
    
    spots1 = [station_neighbors1[i] for i in range(numVisitors)]
    spots2 = [station_neighbors2[i] for i in range(numVisitors)]

    horizonCons = []
    robotIdx = range(numRobots)

    for k in range(visitStart, visitDeadline-visit1Duration-visit2Duration):
        robotCombos = []

        combos = combinations(robotIdx, numVisitors)
        for combo in combos:
            robotCombos.append(
                And(
                    And([ # Visit S1.
                        And(
                            robotPos[k][combo[j]] == int(spots1[j]),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([ # Stay at S1.
                        And(
                            robotPos[k+n][combo[j]] == int(spots1[j]),
                            occupied[k+n][combo[j]]
                        )
                        for j in range(numVisitors)
                        for n in range(1, visit1Duration)
                    ]),
                    Or([ # After S1.
                        And(
                            And([ # Visit S2.
                                And(
                                    robotPos[k+l][combo[j]] == int(spots2[j]),
                                    occupied[k+l][combo[j]]
                                )
                                for j in range(numVisitors)
                            ]),
                            And([ # Stay at S2
                                And([
                                    And(
                                        robotPos[k+l+p][combo[j]] == int(spots2[j]),
                                        occupied[k+l+p][combo[j]]
                                    )
                                    for j in range(numVisitors)
                                ])
                                for p in range(1, visit2Duration)
                            ]),
                            And([ # Keep occupied from S1 to S2.
                                occupied[k+m][combo[j]]
                                for m in range(1, l)
                                for j in range(numVisitors)
                            ]),
                            And([ # Unoccupied after S2 visit.
                                Not(occupied[k+l+visit2Duration][combo[j]]) 
                                for j in range(numVisitors)
                            ])
                        )
                        for l in range(visit1Duration+1, visitDeadline-k-visit2Duration)
                    ])
                )
            )
        horizonCons.append(Or(robotCombos))
    solver.add(Or(horizonCons))

def n_robots_sequence_three_visits(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    grid: np.ndarray, obstacles: list, stations: list,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station1: list, station2: list, station3: list,
    visit1Duration: int, visit2Duration: int, visit3Duration: int
):
    '''Constraint for n robots to sequence three diff duration visits.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

    station_neighbors1 = get_neighbors(station1[0], grid, obstacles, stations)
    station_neighbors2 = get_neighbors(station2[0], grid, obstacles, stations)
    station_neighbors3 = get_neighbors(station3[0], grid, obstacles, stations)

    check_enough_neighbors(station1, station_neighbors1, numVisitors)
    check_enough_neighbors(station2, station_neighbors2, numVisitors)
    check_enough_neighbors(station3, station_neighbors3, numVisitors)
    
    spots1 = [station_neighbors1[i] for i in range(numVisitors)]
    spots2 = [station_neighbors2[i] for i in range(numVisitors)]
    spots3 = [station_neighbors3[i] for i in range(numVisitors)]
    # print(spots1, spots2, spots3)

    horizonCons = []
    robotIdx = range(numRobots)

    for k in range(visitStart, visitDeadline-visit1Duration-visit2Duration):
        robotCombos = []

        combos = combinations(robotIdx, numVisitors)
        for combo in combos:
            # print(combo)
            robotCombos.append(
                And(
                    And([ # Visit S1.
                        And(
                            robotPos[k][combo[j]] == int(spots1[j]),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([ # Stay at S1.
                        And(
                            robotPos[k+n][combo[j]] == int(spots1[j]),
                            occupied[k+n][combo[j]]
                        )
                        for j in range(numVisitors)
                        for n in range(1, visit1Duration)
                    ]),
                    Or([ # After S1.
                        And(
                            And([ # Visit S2.
                                And(
                                    robotPos[k+l][combo[j]] == int(spots2[j]),
                                    occupied[k+l][combo[j]]
                                )
                                for j in range(numVisitors)
                            ]),
                            And([ # Stay at S2.
                                And([
                                    And(
                                        robotPos[k+l+p][combo[j]] == int(spots2[j]),
                                        occupied[k+l+p][combo[j]]
                                    )
                                    for j in range(numVisitors)
                                ])
                                for p in range(1, visit2Duration)
                            ]),
                            And([ # Keep occupied from S1 to S2.
                                occupied[k+m][combo[j]]
                                for m in range(1, l)
                                for j in range(numVisitors)
                            ]),
                            Or([
                                And(
                                    And([ # Visit S3.
                                        And(
                                            robotPos[k+l+o][combo[j]] == int(spots3[j]),
                                            occupied[k+l+o][combo[j]]
                                        )
                                        for j in range(numVisitors)
                                    ]),
                                    And([ # Stay at S3.
                                        And([
                                            And(
                                                robotPos[k+l+o+p][combo[j]] == int(spots3[j]),
                                                occupied[k+l+o+p][combo[j]]
                                            )
                                            for j in range(numVisitors)
                                        ])
                                        for p in range(1, visit3Duration)
                                    ]),
                                    And([ # Keep occupied from S2 to S1.
                                        occupied[k+l+m][combo[j]]
                                        for m in range(1, o)
                                        for j in range(numVisitors)
                                    ]),
                                    And([ # Unoccupied after S3 visit.
                                        Not(occupied[k+l+o+visit3Duration][combo[j]]) 
                                        for j in range(numVisitors)
                                    ])
                                )
                                for o in range(visit2Duration+1, visitDeadline-k-l-visit3Duration)
                            ])
                        )
                        for l in range(visit1Duration+1, visitDeadline-k-visit2Duration-visit3Duration)
                    ])
                )

            )
        horizonCons.append(Or(robotCombos))
    solver.add(Or(horizonCons))

