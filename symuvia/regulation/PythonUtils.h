/* ----------------------------------------------------------------------
* OPEN-SYMUVIA (http://open-symuvia.ifsttar.fr). This file is part of OPEN-SYMUVIA.
*
* OPEN-SYMUVIA is an open-source traffic simulator.
*
* Copyright (C) - IFSTTAR, ENTPE - Ludovic Leclercq, Cécile Bécarie
*
* Open-SymuVia is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
* 
* OPEN-SYMUVIA is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lessed General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA or 
* see <http://ww.gnu.org/licenses/>
*
* For more information, please consult: <http://open-symuvia.ifsttar.fr> or 
* send an email to cecile.becarie@ifsttar.fr
*
* ----------------------------------------------------------------------*/

#pragma once
#ifndef PythonUtilsH
#define PythonUtilsH

#include <xercesc/util/XercesVersion.hpp>

typedef struct _ts PyThreadState;

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
    class DOMElement;
}

namespace boost {
    namespace python {
        class dict;
        namespace api {
            class object;
        }
    }
}

class Logger;

#include <string>
#include <vector>

static const std::string SCRIPTS_DIRECTORY = std::string("scripts");
static const std::string SCRIPTS_SENSOR_MODULE_NAME = std::string("sensors");
static const std::string SCRIPTS_CONDITION_MODULE_NAME = std::string("conditions");
static const std::string SCRIPTS_ACTION_MODULE_NAME = std::string("actions");
static const std::string SCRIPTS_RESTITUTION_MODULE_NAME = std::string("restitutions");
static const std::string SCRIPTS_CAR_FOLLOWING_MODULE_NAME = std::string("car_following");

class PythonUtils
{
public:
    PythonUtils(void);
    virtual ~PythonUtils(void);

    void setLogger(Logger * pLogger);

    std::string getPythonErrorString();    

    void buildDictFromNode(XERCES_CPP_NAMESPACE::DOMNode * pNode, boost::python::dict & rDict);

    void buildNodeFromDict(XERCES_CPP_NAMESPACE::DOMElement* pElement, const boost::python::dict & rDict);

    void importModule(boost::python::api::object & globals, std::string moduleName, std::string modulePath);

    boost::python::api::object *getMainModule();

    // utilisÃ© en cas d'interpreteur python enfant (si on lance les fonctions de SymuBruit depuis un interpreteur python parent)
    void enterChildInterpreter();
    void exitChildInterpreter();
    void lockChildInterpreter();
    void unlockChildInterpreter();

private:
    void initialize();

    void finalize();

private:
    // Contexte python global
    boost::python::api::object *m_pMainModule;

    // ThreadState associÃ© Ã  l'interpreteur python utilisÃ© dans le cas oÃ¹ on lance SymuVia depuis python
    PyThreadState *m_pState;

    Logger * m_pLogger;
};

#endif // PythonUtilsH