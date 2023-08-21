#define PY_SSIZE_T_CLEAN
#include <string>
#include <Python.h>

int main(int argc, char *argv[]) {
    std::string filename = "pytest.py";
    std::string funcname = "init";
    
    Py_Initialize();
    pName = PyUnicode_DecodeFSDefault(filename);
    pModule = PyImport_Import(pName);
    PyDECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, funcname);
        /* pFunc is a new reference */
        
        if (pFunc and PyCallable_Check(pFunc)) {
            pValue = PyObject_CallObject(pFunc);

            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr, "Call failed\n");
                return 1;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", funcname);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", filename);
        return 1;
    }

    if (Py_FinalizeEx() < 0)
        return 120;

    return 0;
}