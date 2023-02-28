'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for adding constraints for the Z3 solver.
'''
from itertools import combinations
from z3 import *


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

def terminal_position_constraints(solver, robotPos, initialPositions, finalPositions):
    '''Specify the initial and final positions of each robot.'''
    numBots = len(robotPos[0])

    solver.add([robotPos[0][i] == initialPositions[i] for i in range(numBots)])
    solver.add([robotPos[-1][i] == finalPositions[i] for i in range(numBots)])

def stay_in_workspace_constraints(solver, robotPos, workspace, planHorizon):
    '''Keep the robots in the workspace.'''
    numBots = len(robotPos[0])
    numStates = len(workspace)

    solver.add([
        And(
            robotPos[k][i] <= numStates,
            robotPos[k][i] > 0 
        )
        for k in range(planHorizon) 
        for i in range(numBots)
    ])

def check_enough_agents(numRobots: int, numVisitors: int):
    '''Check whether we have enough robots to complete a task.'''
    # TODO: Station has to have at least as many neighbors as robots (for now).
    if numRobots < numVisitors:
        raise ValueError(f"There are not enough robots ({numRobots}) "
                    f"to complete the task specified for {numVisitors} "
                    f"agents.")

def n_robots_visit_station_for_duration(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station: int, visitDuration: int
):
    '''Builds constraints for n robots to visit a station for a duration.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

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
                            robotPos[k][combo[j]] == int(station),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([
                        And(
                            robotPos[k+l][combo[j]] == int(station),
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

def n_robots_transport(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station1: int, station2: int
):
    '''Builds a transport task.
 
    Sets that the robots should visit the source station then destination
    for 1 step each.
    '''
    n_robots_sequence_two_visits(
        solver, robotPos, occupied, visitStart, visitDeadline,
        numVisitors, station1, station2, 1, 1
    )

def n_robots_sequence_two_visits(
    solver: Solver, robotPos: ArithRef, occupied: BoolRef,
    visitStart: int, visitDeadline: int, numVisitors: int,
    station1: int, station2: int,
    visit1Duration: int, visit2Duration: int
):
    '''Builds constraints for n robots to sequence two diff duration visits.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

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
                            robotPos[k][combo[j]] == int(station1),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([ # Stay at S1.
                        And(
                            robotPos[k+n][combo[j]] == int(station1),
                            occupied[k+n][combo[j]]
                        )
                        for j in range(numVisitors)
                        for n in range(1, visit1Duration)
                    ]),
                    Or([ # After S1.
                        And(
                            And([ # Visit S2.
                                And(
                                    robotPos[k+l][combo[j]] == int(station2),
                                    occupied[k+l][combo[j]]
                                )
                                for j in range(numVisitors)
                            ]),
                            And([ # Stay at S2.
                                And([
                                    And(
                                        robotPos[k+l+p][combo[j]] == int(station2),
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
    visitStart: int, visitDeadline: int, numVisitors: int,
    station1: list, station2: list, station3: list,
    visit1Duration: int, visit2Duration: int, visit3Duration: int
):
    '''Constraint for n robots to sequence three diff duration visits.'''
    numRobots = len(robotPos[0])
    check_enough_agents(numRobots, numVisitors)

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
                            robotPos[k][combo[j]] == int(station1),
                            Not(occupied[k][combo[j]])
                        )
                        for j in range(numVisitors)
                    ]),
                    And([ # Stay at S1.
                        And(
                            robotPos[k+n][combo[j]] == int(station1),
                            occupied[k+n][combo[j]]
                        )
                        for j in range(numVisitors)
                        for n in range(1, visit1Duration)
                    ]),
                    Or([ # After S1.
                        And(
                            And([ # Visit S2.
                                And(
                                    robotPos[k+l][combo[j]] == int(station2),
                                    occupied[k+l][combo[j]]
                                )
                                for j in range(numVisitors)
                            ]),
                            And([ # Stay at S2.
                                And([
                                    And(
                                        robotPos[k+l+p][combo[j]] == int(station2),
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
                            Or([ # After S2.
                                And(
                                    And([ # Visit S3.
                                        And(
                                            robotPos[k+l+o][combo[j]] == int(station3),
                                            occupied[k+l+o][combo[j]]
                                        )
                                        for j in range(numVisitors)
                                    ]),
                                    And([ # Stay at S3.
                                        And([
                                            And(
                                                robotPos[k+l+o+p][combo[j]] == int(station3),
                                                occupied[k+l+o+p][combo[j]]
                                            )
                                            for j in range(numVisitors)
                                        ])
                                        for p in range(1, visit3Duration)
                                    ]),
                                    And([ # Keep occupied from S2 to S3.
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

