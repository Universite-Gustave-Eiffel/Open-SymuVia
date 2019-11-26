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
#include "RegulationAction.h"

#include "RegulationCondition.h"
#include "PythonUtils.h"

#include "../reseau.h"
#include "../SystemUtil.h"
#include "../Logger.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/python/exec.hpp>

#include <boost/serialization/vector.hpp>

using namespace boost::python;

XERCES_CPP_NAMESPACE_USE


RegulationAction::RegulationAction(void)
{
}


RegulationAction::~RegulationAction(void)
{
}

std::string RegulationAction::getBuiltinModuleName()
{
    return SCRIPTS_ACTION_MODULE_NAME;
}

void RegulationAction::initialize(Reseau * pReseau, DOMNode * pNodeAction, const std::vector<RegulationCondition*> & lstConditions)
{
    // Appel Ã  la mÃ©thode mÃ¨re
    RegulationElement::initialize(pReseau, pNodeAction);

    // rÃ©cupÃ©ration de la liste des conditions nÃ©cessaires Ã  l'action
    std::string strTmp;
    GetXmlAttributeValue(pNodeAction, "conditions", strTmp, pReseau->GetLogger());
    std::deque<std::string> lstConditionsID = SystemUtil::split(strTmp, ' ');
    for(size_t iCond = 0; iCond < lstConditionsID.size(); iCond++)
    {
        for(size_t jCond = 0; jCond < lstConditions.size(); jCond++)
        {
            if(!lstConditions[jCond]->getID().compare(lstConditionsID[iCond]))
            {
                m_LstConditions.push_back(lstConditions[jCond]);
                break;
            }
        }
    }
}

// test prÃ©liminaire Ã  l'exÃ©cution de l'action
bool RegulationAction::test()
{
    // on fait, sauf si une des conditions est fausse (si pas de condition : on fait)
    bool bDo = true;
    for(size_t i = 0; i < m_LstConditions.size() && bDo; i++)
    {
        bDo = m_LstConditions[i]->isRemplie();
    }
    return bDo;
}

// exÃ©cution de l'action Ã©tant donnÃ© les conditions passÃ©es en paramÃ¨tres
void RegulationAction::execute()
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
        exec((getModuleName() + "." + m_strFunc + "(conditions,context,network,parameters)\n").c_str(), globals, locals);
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
template void RegulationAction::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RegulationAction::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RegulationAction::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(RegulationElement);

    ar & BOOST_SERIALIZATION_NVP(m_LstConditions);
}