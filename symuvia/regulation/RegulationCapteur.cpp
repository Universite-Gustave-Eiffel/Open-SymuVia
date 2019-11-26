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

#include "stdafx.h"
#include "RegulationCapteur.h"
#include "PythonUtils.h"

#include "../reseau.h"
#include "../SystemUtil.h"
#include "../Logger.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/python/exec.hpp>

using namespace boost::python;

XERCES_CPP_NAMESPACE_USE

RegulationCapteur::RegulationCapteur(void)
{
}


RegulationCapteur::~RegulationCapteur(void)
{
}

std::string RegulationCapteur::getBuiltinModuleName()
{
    return SCRIPTS_SENSOR_MODULE_NAME;
}

void RegulationCapteur::initialize(Reseau * pReseau, DOMNode * pNodeCapteur)
{
    // Appel Ã  la mÃ©thode mÃ¨re
    RegulationElement::initialize(pReseau, pNodeCapteur);

    // rÃ©cupÃ©ration du nom de fonction correspondant Ã  la fonction du capteur
    GetXmlAttributeValue(pNodeCapteur, "id", m_strID, pReseau->GetLogger());
}

// mise Ã  jour des mesures du capteur
void RegulationCapteur::update()
{
    try
    {
        object globals = m_pReseau->GetPythonUtils()->getMainModule()->attr("__dict__");
        dict locals;
        locals["network"] = ptr(m_pReseau);
        locals["parameters"] = m_FuncParamDict;
        locals["context"] = m_ContextDict;
        m_Result = eval((getModuleName() + "." + m_strFunc + "(context,network,parameters)\n").c_str(), globals, locals);
    }
    catch( error_already_set )
    {
        m_pReseau->log() << Logger::Error << m_pReseau->GetPythonUtils()->getPythonErrorString() << std::endl;
        m_pReseau->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void RegulationCapteur::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RegulationCapteur::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RegulationCapteur::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(RegulationElement);

    ar & BOOST_SERIALIZATION_NVP(m_strID);
}