#!/usr/bin/env python
import pulp
from pulp.tests import *

import pulpcbc

def test_pulptests():
    tests = [
            pulpTest001,
            pulpTest010, pulpTest011, pulpTest012, pulpTest013, # pulpTest014,
            pulpTest015, pulpTest016, pulpTest017,
            pulpTest018,
            pulpTest020,
            pulpTest030,
            pulpTest040,
            #pulpTest050, TODO: Infeasible
            #pulpTest060,
            pulpTest070, pulpTest075,
            pulpTest080,
            pulpTest090,
            pulpTest100,
            pulpTest110,
            pulpTest120, pulpTest121, pulpTest122, pulpTest123
            ]
    for t in tests:
        t(pulpcbc.CBC())

def test_callback():
    p = pulp.LpProblem()
    x = pulp.LpVariable('x', 0, 1e20, LpInteger)
    y = pulp.LpVariable('y', 0, 1e20, LpInteger)

    p += x + 0.5 * y
    p += 3*x + y >= 8
    p += 1*x + 1.5*y <= 12

    solver = pulpcbc.CBC()

    USED_CB = False
    def event_cb(eventWhere):
        USED_CB = True
        if eventWhere in (pulpcbc.Event.solution, pulpcbc.Event.heuristicSolution):
            solver.getVariable(x)

        return pulpcbc.Action.noAction

    p.solve(solver, callback=event_cb)
    print x.varValue, y.varValue

    assert USED_CB is True

if __name__ == '__main__':
    test_pulptests()
    #test_callback()
