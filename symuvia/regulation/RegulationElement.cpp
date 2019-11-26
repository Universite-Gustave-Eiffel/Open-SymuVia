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
#include "RegulationElement.h"

#include "PythonUtils.h"

#include "../reseau.h"
#include "../SystemUtil.h"
#include "../Logger.h"

#include "../Xerces/XMLUtil.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/python/exec.hpp>

using namespace boost::python;

XERCES_CPP_NAMESPACE_USE


RegulationElement::RegulationElement(void)
{
    m_pReseau = NULL;
}


RegulationElement::~RegulationElement(void)
{
}

std::string RegulationElement::getModuleName()
{
    if(!m_strModule.compare(""))
    {
        return getBuiltinModuleName();
    }
    else
    {
        return SystemUtil::GetFileName(m_strModule);
    }
}

void RegulationElement::initialize(Reseau * pReseau, DOMNode * pNodeElement)
{
    m_pReseau = pReseau;

    // rÃ©cupÃ©ration du nom de fonction correspondant Ã  la fonction de l'Ã©lÃ©ment
    GetXmlAttributeValue(pNodeElement, "fonction", m_strFunc, pReseau->GetLogger());

    // rÃ©cupÃ©ration du chemin du module contenant la fonction de l'Ã©lÃ©ment
    if(GetXmlAttributeValue(pNodeElement, "module", m_strModule, pReseau->GetLogger()))
    {
        // Chargement du module
        try
        {
            object globals = m_pReseau->GetPythonUtils()->getMainModule()->attr("__dict__");
            exec("import imp\n", globals);
            m_pReseau->GetPythonUtils()->importModule(globals, getModuleName(), m_strModule);
        }
        catch( error_already_set )
        {
            m_pReseau->log() << Logger::Error << m_pReseau->GetPythonUtils()->getPythonErrorString() << std::endl;
            m_pReseau->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
    }

    // construction du dictionnaire correspondant aux paramÃ¨tres de la fonction de l'Ã©lÃ©ment
    // rÃ©cupÃ©ration du sous noedu PARAMETRES
    DOMNode * pNodeParametres = m_pReseau->GetXMLUtil()->SelectSingleNode("PARAMETRES", pNodeElement->getOwnerDocument(), (DOMElement*)pNodeElement);
    if(pNodeParametres)
    {
        m_pReseau->GetPythonUtils()->buildDictFromNode(pNodeParametres, m_FuncParamDict);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void RegulationElement::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RegulationElement::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RegulationElement::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_pReseau);
    ar & BOOST_SERIALIZATION_NVP(m_strFunc);
    ar & BOOST_SERIALIZATION_NVP(m_strModule);
    ar & BOOST_SERIALIZATION_NVP(m_FuncParamDict);
    ar & BOOST_SERIALIZATION_NVP(m_ContextDict);
}