import pulp
import pulpcbc

p = pulp.LpProblem()
x = pulp.LpVariable('x', 0, 1e100, 'Integer')
y = pulp.LpVariable('y', 0, 1e100, 'Integer')
p += x + 0.5 * y
p += 3*x + y >= 8
p += 1*x + 1.5*y <= 12

solver = pulpcbc.CBC()

def event_cb(eventWhere):
    if eventWhere in (pulpcbc.Event.solution, pulpcbc.Event.heuristicSolution):
        print solver.getVariable(x)

    return pulpcbc.Action.noAction

p.solve(solver, callback=event_cb)

print x.varValue
print y.varValue
