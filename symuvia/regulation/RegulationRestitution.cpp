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
#include "RegulationRestitution.h"

#include "RegulationCondition.h"
#include "PythonUtils.h"

#include "../reseau.h"
#include "../TraceDocTrafic.h"
#include "../XMLDocTrafic.h"
#include "../Logger.h"

#include "Xerces/XMLUtil.h"

#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>

#include <boost/python/exec.hpp>
#include <boost/python/extract.hpp>

using namespace boost::python;

XERCES_CPP_NAMESPACE_USE


RegulationRestitution::RegulationRestitution(void)
{
}


RegulationRestitution::~RegulationRestitution(void)
{
}

std::string RegulationRestitution::getBuiltinModuleName()
{
    return SCRIPTS_RESTITUTION_MODULE_NAME;
}

void RegulationRestitution::initialize(Reseau * pReseau, DOMNode * pNodeRestitution, const std::vector<RegulationCondition*> & lstConditions)
{
    // Appel Ã  la mÃ©thode mÃ¨re
    RegulationElement::initialize(pReseau, pNodeRestitution);

    // Dans le cas des restitutions, on met toutes les conditions (on porura ainsi se servir de tous ces contextes pour la restitution)
    m_LstConditions.insert(m_LstConditions.end(), lstConditions.begin(), lstConditions.end());
}


// exÃ©cution de l'action de restitution
void RegulationRestitution::restitution(TraceDocTrafic * pDocTrafic, const std::string & briqueID)
{
    try
    {
        object globals = m_pReseau->GetPythonUtils()->getMainModule()->attr("__dict__");
        dict locals;

        // Construction de la liste des contextes des conditions associÃ©es
        dict conditions;
        for(size_t i = 0; i < m_LstConditions.size(); i++)
        {
            conditions[m_LstConditions[i]->getID()] = m_LstConditions[i]->getContext();
        }

        locals["conditions"] = conditions;
        locals["network"] = ptr(m_pReseau);
        locals["parameters"] = m_FuncParamDict;
        locals["context"] = m_ContextDict;
        object result = eval((getModuleName() + "." + m_strFunc + "(conditions,context,network,parameters)\n").c_str(), globals, locals);

        // construction du noeud Ã  partir du resultat
        if(!result.is_none() && pDocTrafic->GetXMLDocTrafic())
        {
            DOMElement* pNodeRegulation = pDocTrafic->GetXMLDocTrafic()->getTemporaryInstantXMLDocument()->createElement(XS("REGULATION"));
            pNodeRegulation->setAttribute(XS("id"), XS(briqueID.c_str()));
            m_pReseau->GetPythonUtils()->buildNodeFromDict(pNodeRegulation, extract<dict>(result));
            pDocTrafic->AddSimuRegulation(pNodeRegulation);
        }
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
template void RegulationRestitution::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RegulationRestitution::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RegulationRestitution::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(RegulationAction);
}