#include <coin/CbcEventHandler.hpp>

typedef CbcEventHandler::CbcAction (*CBType)(PyObject*, CbcModel *, CbcEventHandler::CbcEvent);

class CyEventHandler : public CbcEventHandler
{
    public:
        CyEventHandler(PyObject *obj, CBType event_cb);
        virtual ~CyEventHandler();

        virtual CbcEventHandler::CbcAction event(CbcEvent whichEvent);
        virtual CbcEventHandler *clone() const;

    private:
        PyObject *_obj;
        CBType _event_cb;
};
