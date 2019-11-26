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
#include "RegulationModule.h"

#include "PythonUtils.h"
#include "RegulationBrique.h"

#include "../reseau.h"

#include "../Xerces/XMLUtil.h"

#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMNodeList.hpp>

#include <boost/serialization/vector.hpp>

using namespace boost::python;

XERCES_CPP_NAMESPACE_USE

RegulationModule::RegulationModule(void)
{
    m_pReseau = NULL;
    m_pBriqueActive = NULL;
}

RegulationModule::RegulationModule(Reseau * pReseau)
{
    m_pReseau = pReseau;
    m_pBriqueActive = NULL;
}


RegulationModule::~RegulationModule(void)
{
    if(m_LstBriquesRegulation.size() > 0)
    {
        m_pReseau->GetPythonUtils()->lockChildInterpreter();
        for(size_t i = 0; i < m_LstBriquesRegulation.size(); i++)
        {
            delete m_LstBriquesRegulation[i];
        }
        m_pReseau->GetPythonUtils()->unlockChildInterpreter();
    }
}

// exÃ©cute les briques de rÃ©gulation pour le pas de temps Ã©coulÃ©
void RegulationModule::run()
{
    if(m_LstBriquesRegulation.size() > 0)
    {
        m_pReseau->GetPythonUtils()->enterChildInterpreter();
        // traitement de chaque brique
        for(size_t iBrique = 0; iBrique < m_LstBriquesRegulation.size(); iBrique++)
        {
            m_pBriqueActive = m_LstBriquesRegulation[iBrique];
            m_LstBriquesRegulation[iBrique]->run();
            m_pBriqueActive = NULL;
        }
        m_pReseau->GetPythonUtils()->exitChildInterpreter();
    }
}


// restitution des sorties Ã©ventuelles dans le fichier de trafic
void RegulationModule::restitution(TraceDocTrafic * pDocTrafic)
{
    if(m_LstBriquesRegulation.size() > 0)
    {
        m_pReseau->GetPythonUtils()->enterChildInterpreter();
        // traitement de chaque brique
        for(size_t iBrique = 0; iBrique < m_LstBriquesRegulation.size(); iBrique++)
        {
            m_LstBriquesRegulation[iBrique]->restitution(pDocTrafic);
        }
        m_pReseau->GetPythonUtils()->exitChildInterpreter();
    }
}

// Instancie une nouvelle brique de rÃ©gulation
void RegulationModule::addBrique(DOMNode * pNodeBrique)
{
    m_pReseau->GetPythonUtils()->enterChildInterpreter();

    // RÃ©cupÃ©ration de l'ID de la brique
    std::string strTmp;
    GetXmlAttributeValue(pNodeBrique, "id", strTmp, m_pReseau->GetLogger());

    // RÃ©cupÃ©ration du taux de respect de la consigne
    double dbTmp = 1.0;
    GetXmlAttributeValue(pNodeBrique, "taux_respect_consigne", dbTmp, m_pReseau->GetLogger());

    // Instanciation de la nouvelle brique de rÃ©gulation
    RegulationBrique * pBrique = new RegulationBrique(m_pReseau, strTmp, dbTmp);
    m_LstBriquesRegulation.push_back(pBrique);

    // Ajout de dÃ©tecteurs
    DOMNode * pXMLNodeDetecteurs = m_pReseau->GetXMLUtil()->SelectSingleNode(".//DETECTEURS", pNodeBrique->getOwnerDocument(), (DOMElement*)pNodeBrique);

    if(pXMLNodeDetecteurs)
    {
		XMLSize_t counti = pXMLNodeDetecteurs->getChildNodes()->getLength();
		for(XMLSize_t i=0; i<counti;i++)
        {      
			DOMNode *pXMLDetecteur = pXMLNodeDetecteurs->getChildNodes()->item(i);
			if (pXMLDetecteur->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            pBrique->addDetecteur(pXMLDetecteur);
        }
    }

    // Ajout de dÃ©clencheurs
    DOMNode * pXMLNodeDeclencheurs = m_pReseau->GetXMLUtil()->SelectSingleNode(".//DECLENCHEURS", pNodeBrique->getOwnerDocument(), (DOMElement*)pNodeBrique);

    if(pXMLNodeDeclencheurs)
    {
		XMLSize_t counti = pXMLNodeDeclencheurs->getChildNodes()->getLength();
		for(XMLSize_t i=0; i<counti;i++)
        {      
			DOMNode *pXMLDeclencheur = pXMLNodeDeclencheurs->getChildNodes()->item(i);
			if (pXMLDeclencheur->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            pBrique->addDeclencheur(pXMLDeclencheur);
        }
    }

    // Ajout des actions
    DOMNode * pXMLNodeActions = m_pReseau->GetXMLUtil()->SelectSingleNode(".//ACTIONS", pNodeBrique->getOwnerDocument(), (DOMElement*)pNodeBrique);

    if(pXMLNodeActions)
    {
		XMLSize_t counti = pXMLNodeActions->getChildNodes()->getLength();
		for(XMLSize_t i=0; i<counti;i++)
        {      
			DOMNode *pXMLAction = pXMLNodeActions->getChildNodes()->item(i);
			if (pXMLAction->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            pBrique->addAction(pXMLAction);
        }
    }

    // Ajout des restitutions
    DOMNode * pXMLNodeRestitutions = m_pReseau->GetXMLUtil()->SelectSingleNode(".//RESTITUTIONS", pNodeBrique->getOwnerDocument(), (DOMElement*)pNodeBrique);

    if(pXMLNodeRestitutions)
    {
		XMLSize_t counti = pXMLNodeRestitutions->getChildNodes()->getLength();
		for(XMLSize_t i=0; i<counti;i++)
        {      
			DOMNode *pXMLRestitution = pXMLNodeRestitutions->getChildNodes()->item(i);
			if (pXMLRestitution->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            pBrique->addRestitution(pXMLRestitution);
        }
    }

    m_pReseau->GetPythonUtils()->exitChildInterpreter();
}

// renvoie la brique de rÃ©gulation dont les actions sont en cours d'exÃ©cution
RegulationBrique * RegulationModule::getBriqueActive()
{
    return m_pBriqueActive;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void RegulationModule::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RegulationModule::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RegulationModule::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_pReseau);
    ar & BOOST_SERIALIZATION_NVP(m_LstBriquesRegulation);
    ar & BOOST_SERIALIZATION_NVP(m_pBriqueActive);
}