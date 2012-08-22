#include <Python.h>
#include <stdio.h>

#include "CyEventHandler.h"

CyEventHandler::CyEventHandler(PyObject *obj, CBType event_cb) :
    _obj(obj),
    _event_cb(event_cb)
{
    Py_XINCREF(_obj);
}

CyEventHandler::~CyEventHandler()
{
    Py_XDECREF(_obj);
}

CbcEventHandler::CbcAction CyEventHandler::event(CbcEvent whichEvent)
{
    return _event_cb(_obj, getModel(), whichEvent);
}

CbcEventHandler *CyEventHandler::clone() const
{
    return new CyEventHandler((PyObject*)_obj, (CBType)_event_cb);
}
