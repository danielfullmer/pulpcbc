from libc.stdlib cimport malloc, free
from cython.operator import dereference
from cpython.ref cimport PyObject
from libcpp cimport bool

from pulp import *

# TODO: Multithreaded callbacks may change _cb_model? Fix this.

cdef extern from "coin/OsiSolverInterface.hpp":
    cdef cppclass OsiSolverInterface:
        void loadProblem(int numcols, int numrows,
                         int *start, int *index, double *value,
                         double *collb, double *colbub, double *obj,
                         char *rowsen, double *rowrhs, double *rowrng)
        void setObjSense(double)
        void setInteger(int)
        double *getColSolution()

cdef extern from "coin/OsiClpSolverInterface.hpp":
    cdef cppclass OsiClpSolverInterface(OsiSolverInterface):
        OsiClpSolverInterface()

cdef extern from "coin/CbcModel.hpp":
    cdef cppclass CbcModel:
        CbcModel(OsiSolverInterface)

        void initialSolve() nogil
        void branchAndBound(int) nogil
        void passInEventHandler(CbcEventHandler*)
        OsiSolverInterface *solver()

        double *bestSolution()
        double *getColSolution()
        double getObjValue()
        double getBestPossibleObjValue()
        int getSolutionCount()
        int getIterationCount()
        int getNodeCount()
        bool isProvenOptimal()
        bool isProvenInfeasible()
        bool isContinuousUnbounded()
        CbcModel *parentModel()
        void setSpecialOptions(int)
        int specialOptions()

        int getNumCols()
        double *getCbcColSolution()

cdef extern from "coin/CbcSolver.hpp":
    cdef void CbcMain0(CbcModel) nogil
    cdef void CbcMain1(int, char**, CbcModel) nogil

    cdef cppclass CbcSolver:
        pass

cdef extern from * namespace "CbcEventHandler":
    enum CbcEvent:
        node = 200
        treeStatus
        solution
        heuristicSolution
        beforeSolution1
        beforeSolution2
        afterHeuristic
        endSearch

    enum CbcAction:
        noAction = -1
        stop = 0
        processing
        restart
        restartRoot
        addCuts
        killSolution

# For pure-python convenience
class Event(int):
    node = 200
    treeStatus = 201
    solution = 202
    heuristicSolution = 203
    beforeSolution1 = 204
    beforeSolution2 = 205
    afterHeuristic = 206
    endSearch = 207

class Action(int):
    noAction = -1
    stop = 0
    processing = 1
    restart = 2
    restartRoot = 3
    addCuts = 4
    killSolution = 5

cdef extern from "coin/CbcEventHandler.hpp":
    cdef cppclass CbcEventHandler:
        CbcAction event(CbcEvent)


ctypedef CbcAction (*CBType)(PyObject*, CbcModel *, CbcEvent)

cdef extern from "CyEventHandler.h":
    cdef cppclass CyEventHandler(CbcEventHandler):
        CyEventHandler(PyObject *obj, CBType event_cb)

cdef CbcAction run_event_cb(PyObject *obj, CbcModel *model, CbcEvent whichEvent) with gil:
    cdef CBC self = <CBC>obj

    if model.parentModel() != NULL:
        return Action.noAction

    self._cb_model = model
    return self.callback(whichEvent)

cdef class CBC:
    cdef CbcModel *_cb_model
    cdef callback
    cdef OsiClpSolverInterface *_solver
    cdef public mip, options
    cdef v2n, vname2n, n2v, c2n, n2c

    def __init__(self, mip=True, options=[], *args, **kwargs):
        self.mip = mip
        self.options = options

    def available(self):
        """True if the solver is available"""
        raise True

    def solve(self, lp, **kwargs):
        """Solve the problem lp"""
        # Always go through the solve method of LpProblem
        return lp.solve(self, **kwargs)

    def actualSolve(self, lp, callback=None):
        LpObjSenses = {LpMaximize : -1,
                       LpMinimize : 1}

        variables = lp.variables()
        constraints = lp.constraints

        cdef double objSense = LpObjSenses[lp.sense]
        cdef int numVars = len(variables)
        cdef int numRows = len(constraints)

        cdef double *rhsValues = <double *>malloc(sizeof(double) * numRows)
        cdef double *rangeValues = <double *>malloc(sizeof(double) * numRows)
        cdef char *rowType = <char *>malloc(sizeof(char) * numRows)

        self.v2n = dict(((variables[i],i) for i in range(numVars)))
        self.n2v = dict((i,variables[i]) for i in range(numVars))
        self.vname2n = dict()
        for i in range(numVars):
            vname = variables[i].name
            if vname in self.vname2n:
                raise PulpError('Found duplicated variable name.')
            else:
                self.vname2n[vname] = i

        self.c2n = {}
        self.n2c = {}
        for i, c in enumerate(constraints):
            rhsValues[i] = -constraints[c].constant
            #for ranged constraints a<= constraint >=b
            rangeValues[i] = 0.0
            sense = constraints[c].sense
            if sense == LpConstraintEQ:
                rowType[i] = 'E'
            elif sense == LpConstraintLE:
                rowType[i] = 'L'
            elif sense == LpConstraintGE:
                rowType[i] = 'G'
            self.c2n[c] = i
            self.n2c[i] = c

        # Constraint Matrix
        coeffs = lp.coefficients()
        sparseMatrix = sparse.Matrix(range(numRows), range(numVars))
        for var,row,coeff in coeffs:
            sparseMatrix.add(self.c2n[row], self.vname2n[var], coeff)
        (numels, mystartsBase, mylenBase, myindBase,
         myelemBase) = sparseMatrix.col_based_arrays()

        cdef int *startsBase = <int *>malloc(sizeof(int) * len(mystartsBase))
        for i from 0 <= i < len(mystartsBase):
            startsBase[i] = mystartsBase[i]

        cdef int *indBase = <int *>malloc(sizeof(int) * len(myindBase))
        for i from 0 <= i < len(myindBase):
            indBase[i] = myindBase[i]

        cdef double *elemBase = <double *>malloc(sizeof(double) * len(myelemBase))
        for i from 0 <= i < len(myelemBase):
            elemBase[i] = myelemBase[i]

        
        # Row bounds
        cdef double *lowerBounds = <double *>malloc(sizeof(double) * numVars)
        cdef double *upperBounds = <double *>malloc(sizeof(double) * numVars)
        for v in variables:
            if v.lowBound is not None:
                lowerBounds[self.v2n[v]] = v.lowBound
            else:
                lowerBounds[self.v2n[v]] = -1e100
            if v.upBound is not None:
                upperBounds[self.v2n[v]] = v.upBound
            else:
                upperBounds[self.v2n[v]] = 1e100
            assert lowerBounds[self.v2n[v]] <= upperBounds[self.v2n[v]]

        # Objective coefficients
        cdef double *objectCoeffs = <double *>malloc(sizeof(double) * numVars)
        for i from 0 <= i < numVars:
            objectCoeffs[i] = 0
        for v,val in lp.objective.iteritems():
            objectCoeffs[self.v2n[v]]=val

        # Solver
        self._solver = solver = new OsiClpSolverInterface()
        solver.loadProblem(numVars, numRows,
                           startsBase, indBase, elemBase,
                           lowerBounds, upperBounds, objectCoeffs,
                           rowType, rhsValues, rangeValues)
        solver.setObjSense(objSense)

        # Set integer columns
        if self.mip:
            for v in lp.variables():
                if v.cat == LpInteger:
                    solver.setInteger(self.v2n[v])

        # Model
        model = new CbcModel(dereference(solver))

        # Disable reduced model
        model.setSpecialOptions(model.specialOptions() & (~512))

        # Create and pass in event handler
        cdef CbcEventHandler *event_handler
        if callback:
            self.callback = callback
            event_handler = new CyEventHandler(
                <PyObject*>self,
                run_event_cb)
            model.passInEventHandler(event_handler)

        # Set options
        options = " ".join('-%s' % opt for opt in self.options)
        options = options.split(" ")
        cdef int argc = len(options) + 3
        cdef char **argv = <char**>malloc(sizeof(char*) * argc)
        argv[0] = "cbc"
        for i, option in enumerate(options):
            argv[i+1] = options[i]
        argv[argc-2] = "-solve"
        argv[argc-1] = "-quit"

        # Call CBC
        with nogil:
            CbcMain0(dereference(model))
            CbcMain1(argc, argv, dereference(model))

        free(argv)

        lp.objValue = model.getObjValue()
        lp.bestBound = model.getBestPossibleObjValue()

        lp.status = LpStatusUndefined

        if model.isProvenOptimal():
            lp.status = LpStatusOptimal

        if model.isProvenInfeasible():
            lp.status = LpStatusInfeasible

        if model.isContinuousUnbounded():
            lp.status = LpStatusUnbounded


        # Set python variables
        cdef double *solution = <double *>model.bestSolution()
        if solution != NULL:
            for i from 0 <= i < numVars:
                self.n2v[i].varValue = solution[i]

        if callback:
            free(<void *>event_handler)
        free(<void *>rhsValues)
        free(<void *>rangeValues)
        free(<void *>rowType)
        free(<void *>startsBase)
        free(<void *>indBase)
        free(<void *>lowerBounds)
        free(<void *>objectCoeffs)
        del model
        del solver

        return lp.status

    def actualResolve(self, lp, **kwargs):
        """
        uses existing problem information and solves the problem
        If it is not implelemented in the solver
        just solve again
        """
        self.actualSolve(lp, **kwargs)

    def getObjValue(self):
        return self._cb_model.getObjValue()

    def getBestPossibleObjValue(self):
        return self._cb_model.getBestPossibleObjValue()

    def getSolutionCount(self):
        return self._cb_model.getSolutionCount()

    def getIterationCount(self):
        return self._cb_model.getIterationCount()

    def getNodeCount(self):
        return self._cb_model.getNodeCount()

    def getNumCols(self):
        return self._cb_model.getNumCols()

    def getVariable(self, var):
        cdef double *solution = self._cb_model.bestSolution()

        return solution[self.v2n[var]]
