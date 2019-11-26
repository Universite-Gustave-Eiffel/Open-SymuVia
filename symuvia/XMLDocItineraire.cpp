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
#include "XMLDocItineraire.h"

#include "SerializeUtil.h"
#include "Xerces/XMLUtil.h"

#include "reseau.h"

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>

#include <boost/archive/xml_woarchive.hpp>
#include <boost/archive/xml_wiarchive.hpp>

// Constructeur par défaut
XMLDocItineraire::XMLDocItineraire()
{
	pXMLDoc = NULL;
    m_pNetwork = NULL;
}

// Constructeur
XMLDocItineraire::XMLDocItineraire(Reseau * pNetwork, XERCES_CPP_NAMESPACE::DOMDocument* pXMLDocument)
{
	pXMLDoc = pXMLDocument;
    m_pNetwork = pNetwork;
}

XMLDocItineraire::~XMLDocItineraire()
{
	pXMLDoc->release();
}

// Initialisation
void XMLDocItineraire::Create()
{
    // Noeud ORIGINES
	m_XmlNodeOrigines = (XERCES_CPP_NAMESPACE::DOMNode*)pXMLDoc->createElement(XS("ORIGINES"));
	XERCES_CPP_NAMESPACE::DOMElement * xmlnoderoot = pXMLDoc->getDocumentElement();
    xmlnoderoot->appendChild( m_XmlNodeOrigines);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void XMLDocItineraire::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void XMLDocItineraire::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void XMLDocItineraire::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_pNetwork);

    SerialiseDOMDocument<Archive>(ar, "pXMLDoc", XS("RESEAU"), pXMLDoc, m_pNetwork->GetXMLUtil());

    // en cas de chargement, on réinitialise le pointeur vers le noeud m_XmlNodeOrigines
    if(Archive::is_loading::value)
    {
        m_XmlNodeOrigines = m_pNetwork->GetXMLUtil()->SelectSingleNode("ORIGINES", pXMLDoc);
    }
}