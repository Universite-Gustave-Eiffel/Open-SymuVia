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
#include "XMLDocTrafic.h"

#include "reseau.h"
#include "symUtil.h"
#include "SystemUtil.h"
#include "TimeUtil.h"
#include "vehicule.h"
#include "tuyau.h"
#include "Xerces/XMLUtil.h"
#include "Xerces/XMLReaderSymuvia.h"
#include "Xerces/DOMLSSerializerSymu.hpp"
#include "SerializeUtil.h"
#include "sensors/SensorsManager.h"
#include "sensors/BlueToothSensor.h"

#include <xercesc/dom/DOMDocument.hpp>

#include <boost/archive/xml_woarchive.hpp>
#include <boost/archive/xml_wiarchive.hpp>

#include <sstream>

XERCES_CPP_NAMESPACE_USE
   


// Constructeur par dÃ©faut
XMLDocTrafic::XMLDocTrafic() : m_bReinitialized(false), m_bAtLeastOneInstant(false)
{
    pXMLDoc = NULL;
    m_pTemporaryInstXMLDoc = NULL;
    m_XmlNodeCreations = NULL;
	m_XmlNodeSorties = NULL;
    m_pCoordTransf = NULL;
    m_XmlNodeInstant = NULL;
    m_XmlWriter = NULL;
    m_pNetwork = NULL;
}

// Constructeur

XMLDocTrafic::XMLDocTrafic(Reseau * pNetwork, DOMDocument * pXMLDocument) : m_bReinitialized(false), m_bAtLeastOneInstant(false)
{
	pXMLDoc = pXMLDocument;
    m_pTemporaryInstXMLDoc = NULL;
    m_XmlNodeCreations = NULL;
	m_XmlNodeSorties = NULL;
    m_pCoordTransf = NULL;
    m_XmlNodeInstant = NULL;
    m_XmlWriter = NULL;
    m_pNetwork = pNetwork;
}

XMLDocTrafic::~XMLDocTrafic()
{
    if(pXMLDoc)
    {
	    pXMLDoc->release();
    }
    if(m_pTemporaryInstXMLDoc)
    {
        m_pTemporaryInstXMLDoc->release();
    }
}

// Initialisation
void XMLDocTrafic::Init(const std::string & strFilename, const std::string & strVer, SDateTime dtDeb, SDateTime dtFin, double dbPerAgrCpts, DOMDocument * xmlDocReseau, unsigned int uiInit, std::string simulationID, std::string traficID, std::string reseauID, OGRCoordinateTransformation *pCoordTransf)
{
	DOMNode			*xmlnoderoot;						
    DOMNode			*xmlnode;		

	m_strFilename = strFilename;

	m_pCoordTransf = pCoordTransf;	// Transformation

    // Fichier de trace        
	m_strTraceFile = strFilename.substr(0, strFilename.length()-3);
    m_strTraceFile += "log";   
	remove(m_strTraceFile.c_str());
    
	// XmlWriter
    if(m_bSave)
    {
		m_XmlWriter = new DOMLSSerializerSymu();
		m_XmlWriter->initXmlWriter(m_strFilename.c_str(), gXMLDecl_ver10, gXMLDecl_encodingASCII);
		m_XmlWriter->setIndent(true);
    }

	// Noeud dÃ©claration
    if(m_bSave)
		m_XmlWriter->writeXmlDeclaration();

    // Noeud OUT (noeud root)
	xmlnoderoot = pXMLDoc->getDocumentElement();

    if (m_bSave)
    {
        m_XmlWriter->writeStartElement(XS("OUT"));

        // Peut Ãªtre nul ici suite aux modification pour ne pas conserver en mÃ©moire les noeuds du XML d'entrÃ©e  (trÃ¨s couteux en RAM si gros fichier)
        // uniquement pour le cas de l'affectation convergente, et on se fiche alors d'Ã©crire le noeud IN donc pas grave (on ne rÃ©cupÃ©rera que 
        // les noeuds INSTANT une fois la pÃ©riode d'affectation validÃ©e).
        if (xmlDocReseau)
        {
            m_XmlWriter->writeStartElement(XS("IN"));

            // Noeud de dÃ©finition des variantes temporelles
            xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("/ROOT_SYMUBRUIT/PLAGES_TEMPORELLES", xmlDocReseau);
            if (xmlnode)
                m_XmlWriter->writeNode(xmlnode);

            // Noeud paramÃ©trage de simulation
            xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("/ROOT_SYMUBRUIT/SIMULATIONS/SIMULATION[@id=\"" + simulationID + "\"]", xmlDocReseau);
            if (xmlnode)
                m_XmlWriter->writeNode(xmlnode);

            // Noeud paramÃ©trage du trafic
            xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("/ROOT_SYMUBRUIT/TRAFICS/TRAFIC[@id=\"" + traficID + "\"]", xmlDocReseau);
            if (xmlnode)
                m_XmlWriter->writeNode(xmlnode);

            // Noeud rÃ©seau
            xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("/ROOT_SYMUBRUIT/RESEAUX/RESEAU[@id=\"" + reseauID + "\"]", xmlDocReseau);
            if (xmlnode)
                m_XmlWriter->writeNode(xmlnode);

            // Noeud SYMUCOM_ELEMENTS
            xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("/ROOT_SYMUBRUIT/SYMUCOM_ELEMENTS", xmlDocReseau);
            if (xmlnode)
                m_XmlWriter->writeNode(xmlnode);

            m_XmlWriter->writeEndElement(); // IN
        }
    }
       
	// Noeud SIMULATION
    m_XmlNodeSimulation = pXMLDoc->createElement(XS("SIMULATION"));
    xmlnoderoot->appendChild(m_XmlNodeSimulation);

    if(m_bSave)
        m_XmlWriter->writeStartElement(XS("SIMULATION"));	

	// attributs		
    // Horodatage de dÃ©but de la simulation       
    if(m_bSave)
		m_XmlWriter->writeAttributeString( XS("time"), XS(SDateTime::Now().ToString().c_str()));

    // Version de l'executable     
    if(m_bSave)
        m_XmlWriter->writeAttributeString( XS("version"), XS(("Symubruit " + strVer).c_str()) );

    // DÃ©but et fin de la simulation
    if(m_bSave)
    {
        m_XmlWriter->writeAttributeString( XS("debut"), XS(dtDeb.ToTime().ToString().c_str() ));
        m_XmlWriter->writeAttributeString( XS("fin"), XS(dtFin.ToTime().ToString().c_str() ));
    } 

	// Seed utilisÃ©
	if(m_bSave)
		m_XmlWriter->writeAttributeString( XS("seed"), XS(SystemUtil::ToString( (int64_t) uiInit).c_str()) );	

	// Noeud VEHICULES		
	m_XmlNodeVehicules = pXMLDoc->createElement(XS("VEHS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeVehicules);

	// Noeud CELLULES		
	m_XmlNodeCellules = pXMLDoc->createElement(XS("SGTS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeCellules);       

    // Noeud ARRETS		
	m_XmlNodeArrets = pXMLDoc->createElement(XS("ARRETS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeArrets);

    // Noeud TRONCONS
    m_XmlNodeTroncons = pXMLDoc->createElement(XS("SYM_TRONCONS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeTroncons);

    // Noeud CONNEXIONS        
    m_XmlNodeConnexions = pXMLDoc->createElement(XS("SYM_CONNEXIONS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeConnexions);

    // Noeud ENTREES
    m_XmlNodeSymEntrees = pXMLDoc->createElement(XS("SYM_ENTREES"));
	m_XmlNodeConnexions->appendChild(m_XmlNodeSymEntrees);

    // Noeud SORTIES
    m_XmlNodeSymSorties = pXMLDoc->createElement(XS("SYM_SORTIES"));
	m_XmlNodeConnexions->appendChild(m_XmlNodeSymSorties);

    // Noeud CAPTEURS (DEFINITION)
    m_XmlNodeSymCpts = pXMLDoc->createElement(XS("SYM_CAPTEURS"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeSymCpts);

    // Noeud CAPTEURS (simulation)
    m_XmlNodeSimuCpts = pXMLDoc->createElement(XS("GESTION_CAPTEUR"));
	m_XmlNodeSimulation->appendChild(m_XmlNodeSimuCpts);

    DOMNode* xmlnodePeriodes;
	xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEUR_GLOBAL"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
	xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_PONCTUELS"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
    xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_EDIE"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
    xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_LONGITUDINAUX"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
    xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_MFD"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
    xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_BLUETOOTH"));
	m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
    xmlnodePeriodes = pXMLDoc->createElement(XS("CAPTEURS_RESERVOIRS"));
    m_XmlNodeSimuCpts->appendChild(xmlnodePeriodes);
	

	// Noeud INSTANTS		
	m_XmlNodeInstants = pXMLDoc->createElement(XS("INSTANTS"));			
	m_XmlNodeSimulation->appendChild(m_XmlNodeInstants);
          
}

// Re-initialisation (aprÃ¨s une dÃ©sÃ©rialisation par exemple)
void XMLDocTrafic::ReInit(std::string results)
{
	// Noeud SIMULATION
    m_XmlNodeSimulation = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("SIMULATION", pXMLDoc);

    // Noeud VEHICULES		
    m_XmlNodeVehicules = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("VEHS", pXMLDoc, m_XmlNodeSimulation);

	// Noeud CELLULES		
    m_XmlNodeCellules = m_pNetwork->GetXMLUtil()->SelectSingleNode("SGTS", pXMLDoc, m_XmlNodeSimulation);

    // Noeud ARRETS		
    m_XmlNodeArrets = m_pNetwork->GetXMLUtil()->SelectSingleNode("ARRETS", pXMLDoc, m_XmlNodeSimulation);

    // Noeud TRONCONS
    m_XmlNodeTroncons = m_pNetwork->GetXMLUtil()->SelectSingleNode("SYM_TRONCONS", pXMLDoc, m_XmlNodeSimulation);

    // Noeud CONNEXIONS        
    m_XmlNodeConnexions = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("SYM_CONNEXIONS", pXMLDoc, m_XmlNodeSimulation);

    // Noeud ENTREES
    m_XmlNodeSymEntrees = m_pNetwork->GetXMLUtil()->SelectSingleNode("SYM_ENTREES", pXMLDoc, m_XmlNodeConnexions);

    // Noeud SORTIES
    m_XmlNodeSymSorties = m_pNetwork->GetXMLUtil()->SelectSingleNode("SYM_SORTIES", pXMLDoc, m_XmlNodeConnexions);

    // Noeud CAPTEURS (DEFINITION)
    m_XmlNodeSymCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("SYM_CAPTEURS", pXMLDoc, m_XmlNodeSimulation);

    // Noeud CAPTEURS (simulation)
    m_XmlNodeSimuCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("GESTION_CAPTEUR", pXMLDoc, m_XmlNodeSimulation);

	// Noeud INSTANTS		
    m_XmlNodeInstants = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("INSTANTS", pXMLDoc, m_XmlNodeSimulation);

    if(m_bSave)
    {
        // rÃ©-ouverture du writer
        m_XmlWriter = new DOMLSSerializerSymu();
	    m_XmlWriter->initXmlWriter(m_strFilename.c_str(), gXMLDecl_ver10, gXMLDecl_encodingASCII);
	    m_XmlWriter->setIndent(true);
        m_XmlWriter->write(results);
        m_XmlWriter->setCurrentIndentLevel(3);
    }

    m_bReinitialized = true;
}

void XMLDocTrafic::PostInit()
{
    if(m_bSave)
    {
        m_XmlWriter->writeNode(m_XmlNodeTroncons);
        m_XmlWriter->writeNode(m_XmlNodeConnexions);
        m_XmlWriter->writeNode(m_XmlNodeSymCpts);
        m_XmlWriter->writeNode(m_XmlNodeArrets);
	    m_XmlWriter->writeStartElement(XS("INSTANTS"));
    }
}

// Mise en veille (prise snapshot pour reprise plus tard)
void XMLDocTrafic::Suspend()
{
    if(m_bSave && m_XmlWriter!=NULL)
    { 
        m_XmlWriter->close();		
	    delete m_XmlWriter;
        m_XmlWriter = NULL;
    }
}

// Fin
void XMLDocTrafic::Terminate()
{					
    if(m_bSave && m_XmlWriter!=NULL)
    {         
        // End "INSTANTS"
        if(!m_bReinitialized)
        {
	        m_XmlWriter->writeEndElement();				        	        
        }
        else
        {
            m_XmlWriter->setCurrentIndentLevel(2);
            m_XmlWriter->writeEndElement(XS("INSTANTS"), !m_bAtLeastOneInstant);	
        }

	    m_XmlWriter->writeNode(m_XmlNodeVehicules);	        // Ecriture noeud "VEHS"
	    m_XmlWriter->writeNode(m_XmlNodeCellules);	        // Ecriture noeud "SGTS"     
        m_XmlWriter->writeNode(m_XmlNodeSimuCpts);          // Ecriture noed "CAPTEURS"             
        
        if(!m_bReinitialized)
        {
	        m_XmlWriter->writeEndElement(); // End "SIMULATION"	
            m_XmlWriter->writeEndElement();	// End "OUT"	
        }
        else
        {
            m_XmlWriter->setCurrentIndentLevel(1);
            m_XmlWriter->writeEndElement(XS("SIMULATION"));	
            m_XmlWriter->writeEndElement(XS("OUT"));
        }
	    m_XmlWriter->close();		
		delete m_XmlWriter;
        m_XmlWriter = NULL;
    }
};

// Suppression du fichier
void XMLDocTrafic::Remove()
{
    if (m_XmlWriter != NULL)
    {
        m_XmlWriter->close();
        delete m_XmlWriter;
        m_XmlWriter = NULL;
    }
    remove(m_strFilename.c_str());
}

// Ajout d'un instant
void XMLDocTrafic::AddInstant(double dbInstant, double dbTimeStep, int nNbVeh )
{		
	DOMAttr* xmlattr;

    if(m_pTemporaryInstXMLDoc)
    {
        m_pTemporaryInstXMLDoc->release();
    }
    m_pTemporaryInstXMLDoc = m_pNetwork->GetXMLUtil()->CreateXMLDocument(XS("INST"));

	// CrÃ©ation du noeud
    m_XmlNodeInstant = m_pTemporaryInstXMLDoc->getDocumentElement();

	// Attribut valeur de l'instant
	xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("val") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbInstant).c_str()));
	m_XmlNodeInstant->setAttributeNode(xmlattr);

	// Attribut nombre de vÃ©hicule prÃ©sents sur le rÃ©seau (au dÃ©but du pas de temps)
	xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("nbVeh") );
	xmlattr->setValue(XS(SystemUtil::ToString(nNbVeh).c_str()));
	m_XmlNodeInstant->setAttributeNode(xmlattr);

    // Noeud CREATIONS
    m_XmlNodeCreations = m_pTemporaryInstXMLDoc->createElement(XS("CREATIONS"));
    m_XmlNodeInstant->appendChild(m_XmlNodeCreations);

	// Noeud SORTIES
	m_XmlNodeSorties = m_pTemporaryInstXMLDoc->createElement(XS("SORTIES"));
	m_XmlNodeInstant->appendChild(m_XmlNodeSorties);

	// Noeud TRAJECTOIRES
	m_XmlNodetraj = m_pTemporaryInstXMLDoc->createElement(XS("TRAJS"));
	m_XmlNodeInstant->appendChild(m_XmlNodetraj);

	// Noeud de flux
    m_XmlNodeStream = m_pTemporaryInstXMLDoc->createElement(XS("STREAMS"));
	m_XmlNodeInstant->appendChild(m_XmlNodeStream);

    // Noeud des tuyaux
    m_XmlNodeLinks = m_pTemporaryInstXMLDoc->createElement(XS("LINKS"));
	m_XmlNodeInstant->appendChild(m_XmlNodeLinks);

	// Noeuds CELLULES
	DOMNode * xmlnodeCels;
	xmlnodeCels = m_pTemporaryInstXMLDoc->createElement(XS("SGTS"));
	m_XmlNodeInstant->appendChild(xmlnodeCels);

    // Noeuds FEUX
	m_XmlNodeFeux = m_pTemporaryInstXMLDoc->createElement(XS("FEUX"));
	m_XmlNodeInstant->appendChild(m_XmlNodeFeux);

	// Noeuds ENTREES
	m_XmlNodeEntrees = m_pTemporaryInstXMLDoc->createElement(XS("ENTREES"));
	m_XmlNodeInstant->appendChild(m_XmlNodeEntrees);

    // Noeuds REGULATIONS
    m_XmlNodeRegulations = m_pTemporaryInstXMLDoc->createElement(XS("REGULATIONS"));
	m_XmlNodeInstant->appendChild(m_XmlNodeRegulations);

    if(m_bTraceStocks)
    {
        // Noeuds Parkings
	    m_XmlNodeParkings = m_pTemporaryInstXMLDoc->createElement(XS("PARKINGS"));
	    m_XmlNodeInstant->appendChild(m_XmlNodeParkings);

        // Noeuds troncons
        m_XmlNodeTronconsStationnement = m_pTemporaryInstXMLDoc->createElement(XS("TRONCONS_STATIONNEMENT"));
	    m_XmlNodeInstant->appendChild(m_XmlNodeTronconsStationnement);
    }	
};

// Ajout d'une pÃ©riode de surveillance de capteurs
void XMLDocTrafic::AddPeriodeCapteur(double dbDeb, double dbFin, const std::string & nodeName)
{
    DOMElement *        xmlnodePeriode;  
    DOMNode *			xmlnodeCpts;  
    DOMAttr*			xmlattr;

    xmlnodePeriode = pXMLDoc->createElement(XS("PERIODE"));

    m_pNetwork->GetXMLUtil()->SelectSingleNode("//" + nodeName, m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts)->appendChild(xmlnodePeriode);

    xmlattr = pXMLDoc->createAttribute( XS("debut") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbDeb).c_str()));
	xmlnodePeriode->setAttributeNode(xmlattr);

    xmlattr = pXMLDoc->createAttribute( XS("fin") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbFin).c_str()));
	xmlnodePeriode->setAttributeNode(xmlattr);

    // cas particulier de capteur global : pas de noeud "CAPTEUR" intermÃ©diaire (il n'y en a forcÃ©ment qu'un)
    if(nodeName.compare("CAPTEUR_GLOBAL"))
    {
        xmlnodeCpts = pXMLDoc->createElement( XS("CAPTEURS") );
        xmlnodePeriode->appendChild(xmlnodeCpts);
    }
}

/*DOMNode**/ XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteur(const std::string & sIdCpt, const std::string & sTypeVeh,
    const std::string & sVitGlob, const std::string & sNbCumGlob, const std::string & sVitVoie, const std::string & sNbCumVoie, const std::string & sFlow,
    bool bExtractVehicleList)
{
    DOMElement*     xmlnodeCpts; 
    DOMNode*        xmlnodeVehs; 
    DOMElement*     xmlnodeCpt; 
	DOMElement*     xmlnode;
    DOMAttr*	    xmlattr;

	// Le noeud du capteur a t'il dÃ©jÃ  Ã©tÃ© crÃ©Ã© ?
	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_PONCTUELS", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

		if(!xmlnodeCpts)
			return NULL;

		std::string ssPath = "./CAPTEUR[@id=\"" + sIdCpt + "\"]";
        xmlnodeCpt = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode(ssPath, xmlnodeCpts->getOwnerDocument(), xmlnodeCpts);
		if( xmlnodeCpt == NULL )
		{
			xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
			xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

			// ID du capteur
			xmlattr = pXMLDoc->createAttribute( XS("id") );
			xmlattr->setValue(XS(sIdCpt.c_str()));
			xmlnodeCpt->setAttributeNode(xmlattr);
		}
	}                
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if(  !xmlnodeCpts	)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }
		std::string ssPath = "./CAPTEUR[@id=\"" + sIdCpt + "\"]";
        xmlnodeCpt = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode(ssPath, xmlnodeCpts->getOwnerDocument(), xmlnodeCpts);	// Test de l'existence du noeud CAPTEUR
		if( xmlnodeCpt == NULL )
		{
			xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
			xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

			// ID du capteur
			xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("id") );
			xmlattr->setValue(XS(sIdCpt.c_str()));
			xmlnodeCpt->setAttributeNode(xmlattr);
		}	        
    }        

	if (sTypeVeh.length() > 0)
	{
		// Le noeud TYPES_VEHICULE existe t'il dÃ©jÃ  ?
        DOMNode *xmlnodeCptTypesVeh = m_pNetwork->GetXMLUtil()->SelectSingleNode("./TYPES_VEHICULE", xmlnodeCpt->getOwnerDocument(), xmlnodeCpt);
		if( !xmlnodeCptTypesVeh)
		{
			xmlnodeCptTypesVeh = xmlnodeCpt->getOwnerDocument()->createElement(XS("TYPES_VEHICULE"));	// CrÃ©ation
			xmlnodeCpt->appendChild(xmlnodeCptTypesVeh);														// Accrochage
		}

		// CrÃ©ation du noeud
        DOMElement *xmlnodeCptTypeVeh = xmlnodeCptTypesVeh->getOwnerDocument()->createElement(XS("TYPE_VEHICULE"));	// CrÃ©ation
		xmlnodeCptTypesVeh->appendChild(xmlnodeCptTypeVeh);															// Accrochage

		// ID du type de vÃ©hicule
        xmlattr = xmlnodeCptTypeVeh->getOwnerDocument()->createAttribute( XS("type") );
		xmlattr->setValue( XS(sTypeVeh.c_str() ));
		xmlnodeCptTypeVeh->setAttributeNode(xmlattr);

		xmlnode = xmlnodeCptTypeVeh;
	}
	else
		xmlnode = xmlnodeCpt;

    // Vitesse globale
    xmlnode->setAttribute(XS("vit_g"), XS(sVitGlob.c_str()));

    // Nombre de vÃ©hicule cumulÃ© global
    xmlnode->setAttribute(XS("nbcum_g"), XS(sNbCumGlob.c_str()));

    // Vitesses voie par voie
    xmlnode->setAttribute(XS("vit_v"), XS(sVitVoie.c_str()));

    // Nombres de vÃ©hicule cumulÃ© global voie par voie
    xmlnode->setAttribute(XS("nbcum_v"), XS(sNbCumVoie.c_str()));

	if (sTypeVeh.length() == 0 && bExtractVehicleList)
	{
		// Noeuds VEHS
        xmlnodeVehs = xmlnodeCpt->getOwnerDocument()->createElement(XS("VEHS"));
		xmlnodeCpt->appendChild(xmlnodeVehs);
	}

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

/*DOMNode**/ XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurLongitudinal(const std::string & sIdCpt, const std::string & sVoieOrigine, const std::string & sVoieDestination, const std::string & sCount)
{
    DOMElement*     xmlnodeCpts; 
    DOMElement*     xmlnodeCpt; 
    DOMAttr*	    xmlattr;

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_LONGITUDINAUX", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

		if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));    // CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);		        // Accrochage

		// ID du capteur
		xmlattr = pXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
	}                
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if(  !xmlnodeCpts	)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }

		xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

		// ID du capteur
		xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
    }        
    
    // Voie d'origine
    xmlattr = pXMLDoc->createAttribute( XS("voie_origine") );
    xmlattr->setValue( XS(sVoieOrigine.c_str() ));
    xmlnodeCpt->setAttributeNode(xmlattr);

    // Voie de destination
    xmlnodeCpt->setAttribute(XS("voie_destination"), XS(sVoieDestination.c_str()));

    // Nb de changement de voie
    xmlnodeCpt->setAttribute(XS("nb_changement_voie"), XS(sCount.c_str()));

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurEdie(const std::string & sIdCpt, const std::string & sConcentrationCum, const std::string & sDebitCum, const std::string & sConcentration, const std::string & sDebit)
{
    DOMElement*     xmlnodeCpts; 
    DOMElement*     xmlnodeCpt; 
    DOMAttr*	    xmlattr;

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_EDIE", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

		if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));    // CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);		        // Accrochage

		// ID du capteur
		xmlattr = pXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
	}                
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if(  !xmlnodeCpts	)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }

		xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

		// ID du capteur
		xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("id" ));
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
    }        
    
    // Concentration sur le troncon
    xmlnodeCpt->setAttribute(XS("concentration_g"), XS(sConcentrationCum.c_str()));

    // DÃ©bit sur le troncon
    xmlnodeCpt->setAttribute(XS("debit_g"), XS(sDebitCum.c_str()));

    // Concentration voie par voie
    xmlnodeCpt->setAttribute(XS("concentration_v"), XS(sConcentration.c_str()));

    // DÃ©bit voie par voie
    xmlnodeCpt->setAttribute(XS("debit_v"), XS(sDebit.c_str()));

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddIndicateursCapteurGlobal(double dbTotalTravelledDistance, double dbTotalTravelledTime, double dbVit, int nNbVeh)
{
	DOMElement*     xmlnodeCpts = NULL; 
    DOMAttr*	    xmlattr;
	DOMElement*     xmlnodePer; 

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEUR_GLOBAL", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodePer = (DOMElement*)xmlnodeCpts->getLastChild();

		// distance_totale_parcourue
		xmlattr = pXMLDoc->createAttribute( XS("distance_totale_parcourue") );
        xmlattr->setValue(XS(SystemUtil::ToString(2, dbTotalTravelledDistance).c_str()));
		xmlnodePer->setAttributeNode(xmlattr);

		//temps_total_passe
		xmlattr = pXMLDoc->createAttribute( XS("temps_total_passe") );
        xmlattr->setValue(XS(SystemUtil::ToString(2, dbTotalTravelledTime).c_str()));
		xmlnodePer->setAttributeNode(xmlattr);

		// Nombre de vÃ©hicule prÃ©sent sur le rÃ©seau au cours de la pÃ©riode d'agrÃ©gation
		xmlattr = pXMLDoc->createAttribute( XS("nbveh") );
		xmlattr->setValue( XS(SystemUtil::ToString(nNbVeh).c_str() ));
		xmlnodePer->setAttributeNode(xmlattr);

		// Vitesse moyenne
		xmlattr = pXMLDoc->createAttribute( XS("vit") );
		xmlattr->setValue( XS(SystemUtil::ToString(2, dbVit).c_str() ));
		xmlnodePer->setAttributeNode(xmlattr);
	}

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpts;
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurGlobal( const std::string & sIdVeh, double dbTpsEcoulement, double dbDistanceParcourue)
{
    DOMElement*     xmlnodePer; 
	DOMElement*     xmlnodeCpts; 
    DOMElement*     xmlnodeCpt = NULL; 
    DOMAttr*	    xmlattr;

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEUR_GLOBAL", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodePer = (DOMElement*)xmlnodeCpts->getLastChild();

		xmlnodeCpt = pXMLDoc->createElement(XS("V"));    // CrÃ©ation
		xmlnodePer->appendChild(xmlnodeCpt);		        // Accrochage

		if( sIdVeh.size() > 0)
		{
			// ID du capteur
			xmlattr = pXMLDoc->createAttribute( XS("id") );
			xmlattr->setValue(XS(sIdVeh.c_str()));
			xmlnodeCpt->setAttributeNode(xmlattr);

			// Temps passÃ© 
			xmlattr = pXMLDoc->createAttribute( XS("temps") );
			xmlattr->setValue(XS(SystemUtil::ToString(2, dbTpsEcoulement).c_str() ));
			xmlnodeCpt->setAttributeNode(xmlattr);

			// Distance parcourue 
			xmlattr = pXMLDoc->createAttribute( XS("distance") );
			xmlattr->setValue( XS(SystemUtil::ToString(2, dbDistanceParcourue).c_str() ));
			xmlnodeCpt->setAttributeNode(xmlattr);
		}
	}                

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurMFD(const std::string & sIdCpt, bool bIncludeStrictData, const std::string & sGeomLength1, const std::string & sGeomLength2, const std::string & sGeomLength3,
        const std::string & sDistanceTotaleParcourue, const std::string & sDistanceTotaleParcourueStricte, const std::string & sTempsTotalPasse, const std::string & sTempsTotalPasseStrict,
        const std::string & sDebitSortie, const std::string &  sIntDebitSortie, const std::string &  sTransDebitSortie, const std::string & sDebitSortieStrict, const std::string & sLongueurDeplacement, const std::string & sLongueurDeplacementStricte,
        const std::string & sVitesseSpatiale, const std::string & sVitesseSpatialeStricte, const std::string & sConcentration, const std::string & sDebit)
{
    DOMElement*     xmlnodeCpts; 
    DOMElement*     xmlnodeCpt; 
    DOMAttr*	    xmlattr;

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_MFD", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

		if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));    // CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);		        // Accrochage

		// ID du capteur
		xmlattr = pXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
	}                
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if(  !xmlnodeCpts	)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }

		xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

		// ID du capteur
		xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
    }        
    
    xmlnodeCpt->setAttribute(XS("longueur_geometrique_barycentre"), XS(sGeomLength1.c_str()));
    xmlnodeCpt->setAttribute(XS("longueur_geometrique_mvtaval"), XS(sGeomLength2.c_str()));
    xmlnodeCpt->setAttribute(XS("longueur_geometrique_stricte"), XS(sGeomLength3.c_str()));

    xmlnodeCpt->setAttribute(XS("distance_totale_parcourue"), XS(sDistanceTotaleParcourue.c_str()));
    if (bIncludeStrictData) xmlnodeCpt->setAttribute(XS("distance_totale_parcourue_stricte"), XS(sDistanceTotaleParcourueStricte.c_str()));

    xmlnodeCpt->setAttribute(XS("temps_total_passe"), XS(sTempsTotalPasse.c_str()));
    if (bIncludeStrictData) xmlnodeCpt->setAttribute(XS("temps_total_passe_strict"), XS(sTempsTotalPasseStrict.c_str()));

    xmlnodeCpt->setAttribute(XS("debit_sortie"), XS(sDebitSortie.c_str()));
	xmlnodeCpt->setAttribute(XS("debit_sortie_int"), XS(sIntDebitSortie.c_str()));
	xmlnodeCpt->setAttribute(XS("debit_sortie_trans"), XS(sTransDebitSortie.c_str()));

    if (bIncludeStrictData) xmlnodeCpt->setAttribute(XS("debit_sortie_strict"), XS(sDebitSortieStrict.c_str()));

    xmlnodeCpt->setAttribute(XS("longueur_deplacement"), XS(sLongueurDeplacement.c_str()));
    if (bIncludeStrictData) xmlnodeCpt->setAttribute(XS("longueur_deplacement_stricte"), XS(sLongueurDeplacementStricte.c_str()));

    xmlnodeCpt->setAttribute(XS("vitesse_spatiale"), XS(sVitesseSpatiale.c_str()));
    if (bIncludeStrictData) xmlnodeCpt->setAttribute(XS("vitesse_spatiale_stricte"), XS(sVitesseSpatialeStricte.c_str()));

    // Pour compatibilitÃ© avec SymuPlot et SymuPlayer
    xmlnodeCpt->setAttribute(XS("debit"), XS(sDebit.c_str()));
    xmlnodeCpt->setAttribute(XS("concentration"), XS(sConcentration.c_str()));

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurReservoir(const std::string & sIdCpt)
{
    DOMElement*     xmlnodeCpts;
    DOMElement*     xmlnodeCpt;
    DOMAttr*	    xmlattr;

    if (m_bSave) // Mode sauvegarde complÃ¨te
    {
        if (!m_XmlNodeSimuCpts)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_RESERVOIRS", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if (!xmlnodeCpts)
            return NULL;

        xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

        if (!xmlnodeCpts)
            return NULL;

        xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));    // CrÃ©ation
        xmlnodeCpts->appendChild(xmlnodeCpt);		        // Accrochage

        // ID du capteur
        xmlattr = pXMLDoc->createAttribute(XS("id"));
        xmlattr->setValue(XS(sIdCpt.c_str()));
        xmlnodeCpt->setAttributeNode(xmlattr);
    }
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if (!xmlnodeCpts)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }

        xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
        xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

        // ID du capteur
        xmlattr = m_pTemporaryInstXMLDoc->createAttribute(XS("id"));
        xmlattr->setValue(XS(sIdCpt.c_str()));
        xmlnodeCpt->setAttributeNode(xmlattr);
    }

    return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

void XMLDocTrafic::AddTraverseeCpt(/*DOMNode **/XMLDocTrafic::PCAPTEUR xmlNodeCptA, const std::string & vehId, const std::string & entryTime, const std::string & exitTime,
    const std::string & travelledDistance, const std::string & bCreatedInZone, const std::string & bDetroyedInZone)
{
    DOMNode *		 xmlNodeCpt = (DOMNode *)xmlNodeCptA;

    DOMElement * xmlnodeTraversee = xmlNodeCpt->getOwnerDocument()->createElement(XS("TRAVERSEE"));

    xmlNodeCpt->appendChild(xmlnodeTraversee);

    xmlnodeTraversee->setAttribute(XS("veh_id"), XS(vehId.c_str()));
    xmlnodeTraversee->setAttribute(XS("inst_entree"), XS(entryTime.c_str()));
    if (exitTime.compare("-1.000"))
    {
        xmlnodeTraversee->setAttribute(XS("inst_sortie"), XS(exitTime.c_str()));
    }
    xmlnodeTraversee->setAttribute(XS("distance_parcourue"), XS(travelledDistance.c_str()));
    xmlnodeTraversee->setAttribute(XS("creation_interne"), XS(bCreatedInZone.c_str()));
    xmlnodeTraversee->setAttribute(XS("destruction_interne"), XS(bDetroyedInZone.c_str()));
}

XMLDocTrafic::PCAPTEUR XMLDocTrafic::AddInfoCapteurBlueTooth(const std::string & sIdCpt, const BlueToothSensorData& data)
{
    DOMElement*     xmlnodeCpts; 
    DOMElement*     xmlnodeCpt; 
    DOMElement*     xmlnodeAmont; 
    DOMElement*     xmlnodeAval; 
    DOMElement*     xmlnodeVehicules; 
    DOMElement*     xmlnodeVeh; 
    DOMAttr*	    xmlattr;

	if(m_bSave) // Mode sauvegarde complÃ¨te
	{
		if( !m_XmlNodeSimuCpts )
			return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("CAPTEURS_BLUETOOTH", m_XmlNodeSimuCpts->getOwnerDocument(), m_XmlNodeSimuCpts);

        if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpts = (DOMElement*)xmlnodeCpts->getLastChild()->getLastChild();

		if(!xmlnodeCpts)
			return NULL;

		xmlnodeCpt = pXMLDoc->createElement(XS("CAPTEUR"));    // CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);		        // Accrochage

		// ID du capteur
		xmlattr = pXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
	}                
    else	// Mode itÃ©ratif
    {
        if (!m_XmlNodeInstant) // peut arriver si bTrace = false et qu'on termine l'exÃ©cution sur un non multiple de la pÃ©riode d'agrÃ©gation. Dans ce cas (mode itÃ©ratif, on ne fait rien)
            return NULL;

        xmlnodeCpts = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode("./CAPTEURS", m_XmlNodeInstant->getOwnerDocument(), m_XmlNodeInstant);
        if(  !xmlnodeCpts	)	// Test de l'existence du noeud CAPTEURS
        {
            xmlnodeCpts = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEURS")); // CrÃ©ation
            m_XmlNodeInstant->appendChild(xmlnodeCpts);	// Accrochage				
        }

		xmlnodeCpt = m_pTemporaryInstXMLDoc->createElement(XS("CAPTEUR"));	// CrÃ©ation
		xmlnodeCpts->appendChild(xmlnodeCpt);													// Accrochage

		// ID du capteur
		xmlattr = m_pTemporaryInstXMLDoc->createAttribute( XS("id") );
		xmlattr->setValue(XS(sIdCpt.c_str()));
		xmlnodeCpt->setAttributeNode(xmlattr);
    }        
    
    // un noeud fils pour chaque tronÃ§on amont pour lequel on a de la donnÃ©e
    std::map<std::string, std::map<std::string, BlueToothVehSensorData> >::const_iterator iter;
    double dbCoeff;
    for(iter = data.mapCoeffsDir.begin(); iter != data.mapCoeffsDir.end(); iter++)
    {
        std::map<std::string, BlueToothVehSensorData>::const_iterator iter2;
        int nVehsTotal = 0;
        for(iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
        {
            nVehsTotal += iter2->second.nbVehs;
        }

        if(nVehsTotal >= 1)
        {
            // CrÃ©ation du noeud correspondant au tronÃ§on amont
            xmlnodeAmont = xmlnodeCpt->getOwnerDocument()->createElement(XS("TRONCON_AMONT")); // CrÃ©ation
		    xmlnodeCpt->appendChild(xmlnodeAmont); // Accrochage

            // ID du tronÃ§on amont
            xmlattr = xmlnodeAmont->getOwnerDocument()->createAttribute( XS("id") );
            xmlattr->setValue(XS(iter->first.c_str()));
		    xmlnodeAmont->setAttributeNode(xmlattr);

            // tronÃ§ons aval
            for(iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
            {
                xmlnodeAval = xmlnodeAmont->getOwnerDocument()->createElement(XS("TRONCON_AVAL")); // CrÃ©ation
		        xmlnodeAmont->appendChild(xmlnodeAval); // Accrochage

                // ID du tronÃ§on aval
                xmlattr = xmlnodeAval->getOwnerDocument()->createAttribute( XS("id") );
                xmlattr->setValue(XS(iter2->first.c_str()));
		        xmlnodeAval->setAttributeNode(xmlattr);

                // Coefficient directionnel
                dbCoeff = ((double) iter2->second.nbVehs)/nVehsTotal;
                xmlattr = xmlnodeAval->getOwnerDocument()->createAttribute( XS("coeff") );
                xmlattr->setValue(XS(SystemUtil::ToString(2, dbCoeff).c_str()));
		        xmlnodeAval->setAttributeNode(xmlattr);

                // ajout d'un noeud par vÃ©hicule dÃ©tectÃ© pour le mouvement
                xmlnodeVehicules = xmlnodeAval->getOwnerDocument()->createElement(XS("VEHS")); // CrÃ©ation
                xmlnodeAval->appendChild(xmlnodeVehicules); // Accrochage
                for(int iVeh = 0; iVeh < iter2->second.nbVehs; iVeh++)
                {
                    xmlnodeVeh = xmlnodeVehicules->getOwnerDocument()->createElement(XS("VEH")); // CrÃ©ation
                    xmlnodeVehicules->appendChild(xmlnodeVeh); // Accrochage

                    // ID du vÃ©hicule
                    xmlattr = xmlnodeVeh->getOwnerDocument()->createAttribute( XS("id") );
                    xmlattr->setValue(XS(SystemUtil::ToString(iter2->second.nIDs[iVeh]).c_str()));
		            xmlnodeVeh->setAttributeNode(xmlattr);

                    // instant de dÃ©tection d'arrivÃ©e
                    xmlattr = xmlnodeVeh->getOwnerDocument()->createAttribute( XS("instE") );
                    xmlattr->setValue(XS(SystemUtil::ToString(2, iter2->second.dbInstEntrees[iVeh]).c_str()));
		            xmlnodeVeh->setAttributeNode(xmlattr);

                    // instant de dÃ©tection de dÃ©part
                    xmlattr = xmlnodeVeh->getOwnerDocument()->createAttribute( XS("instS") );
                    xmlattr->setValue(XS(SystemUtil::ToString(2, iter2->second.dbInstSorties[iVeh]).c_str()));
		            xmlnodeVeh->setAttributeNode(xmlattr);
                }
            }

        }
    }

	return (XMLDocTrafic::PCAPTEUR)xmlnodeCpt;
}

void XMLDocTrafic::AddVehCpt(/*DOMNode **/XMLDocTrafic::PCAPTEUR xmlNodeCptA, const std::string & sId, double dbInstPsg, int nVoie)
{       
    DOMElement *     xmlnodeVeh; 
    DOMNode *        xmlnodeVehs; 
	DOMNode *		 xmlNodeCpt = (DOMNode *)xmlNodeCptA;

    xmlnodeVehs = xmlNodeCpt->getLastChild();    // Noeud VEHS

    xmlnodeVeh = xmlnodeVehs->getOwnerDocument()->createElement( XS("VEH") );

    xmlnodeVehs->appendChild(xmlnodeVeh);

    // ID du vÃ©hicule
    xmlnodeVeh->setAttribute(XS("id"), XS(sId.c_str()));

    // Instant de passage
    xmlnodeVeh->setAttribute(XS("passage"), XS(SystemUtil::ToString(2, dbInstPsg).c_str()));

    // NumÃ©ro de voie
    xmlnodeVeh->setAttribute(XS("voie"), XS(SystemUtil::ToString(nVoie).c_str()));
}

// Sauvegarde d'un instant
void XMLDocTrafic::SaveLastInstant()
{
    if(m_bSave)
    {
		if( m_XmlWriter->isOpen() )
        {
            m_XmlWriter->writeNode(m_XmlNodeInstant);
            m_bAtLeastOneInstant = true;
        }
    }
}

// Suppression du dernier instant en mÃ©moire DOM
void XMLDocTrafic::RemoveLastInstant()
{
    if(m_pTemporaryInstXMLDoc)
    {
        m_pTemporaryInstXMLDoc->release();
        m_pTemporaryInstXMLDoc = NULL;
        m_XmlNodeInstant = NULL;
    }
}

// Ajoute le contenu du document trafic de la source dans le document courant
void XMLDocTrafic::Add(DocTrafic *docTraficSrc)
{
	XMLDocTrafic* XmlDocTraficSrc = (XMLDocTrafic*)docTraficSrc;
	XmlReaderSymuvia * XmlReader = XmlReaderSymuvia::Create(XmlDocTraficSrc->m_strFilename);
	XmlReader->ReadToFollowing( "INST" );	
	while( XmlReader->Name == "INST" )
	{
		XmlReader->WriteNode(m_XmlWriter, false);		
	}

	// Copie des info capteurs
	// <GESTION_CAPTEUR> / <CAPTEUR...> / <PERIODE> 
	if( XmlDocTraficSrc->m_XmlNodeSimuCpts )
	{
        DOMNodeList* xmlnodetypescapteurs = XmlDocTraficSrc->m_XmlNodeSimuCpts->getChildNodes();
        for (XMLSize_t i=0; i<xmlnodetypescapteurs->getLength(); i++)
	    {
            // import des pÃ©riodes
            DOMNodeList* xmlnodelistPeriodes = xmlnodetypescapteurs->item(i)->getChildNodes();
			if( xmlnodelistPeriodes )
			{
				for (XMLSize_t j=0; j<xmlnodelistPeriodes->getLength(); j++)
				{
					DOMNode* xmlnodeimp = pXMLDoc->importNode(xmlnodelistPeriodes->item(j), true);
                    m_XmlNodeSimuCpts->getChildNodes()->item(i)->appendChild(xmlnodeimp);
				}
			}
        }
	}

	// Copie des info vÃ©hicules
	if( XmlDocTraficSrc->m_XmlNodeVehicules)
	{
		DOMNodeList* xmlnodelistVehs = XmlDocTraficSrc->m_XmlNodeVehicules->getChildNodes();
		if( xmlnodelistVehs )
		{
            std::string		    ssID;
            DOMElement*			xmlnodeVeh;
            DOMAttr*			xmlattr;

            // Construction prÃ©alable d'une map des noeuds VEH dÃ©jÃ  existants (pour Ã©viter plein de SelectSingleNode couteux)
            std::map<std::string, DOMElement*> mapVehs;
            DOMNodeList* xmlnodelistVehsTarget = m_XmlNodeVehicules->getChildNodes();
            for (XMLSize_t i = 0; i < xmlnodelistVehsTarget->getLength(); i++)
            {
                xmlnodeVeh = (DOMElement*)xmlnodelistVehsTarget->item(i);
                ssID = US(xmlnodeVeh->getAttribute(XS("id")));
                mapVehs[ssID] = xmlnodeVeh;
            }

			for (XMLSize_t i=0; i<xmlnodelistVehs->getLength(); i++)
			{
				// Le vÃ©hicule existe t'il dÃ©jÃ  ?
				ssID = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("id")));
                std::map<std::string, DOMElement*>::const_iterator itVeh = mapVehs.find(ssID);
				
                if (itVeh != mapVehs.end())
				{
                    xmlnodeVeh = itVeh->second;

					// Copie uniquement des attributs mise Ã  jour
					// Sortie
					std::string ssSortie = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("sortie")));
                    if(!ssSortie.empty())
                    {
					    xmlattr = pXMLDoc->createAttribute( XS("sortie") );
                        xmlattr->setValue( XS(ssSortie.c_str() ));
					    xmlnodeVeh->setAttributeNode(xmlattr);
                    }

					// Instant de sortie
					std::string ssInstSortie = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("instS")));
					
                    if(!ssInstSortie.empty())
                    {
                        xmlattr = pXMLDoc->createAttribute( XS("instS") );
                        xmlattr->setValue( XS(ssInstSortie.c_str() ));
					    xmlnodeVeh->setAttributeNode(xmlattr);
                    }		

                    // Distance cumulÃ©e
                    std::string ssDstParcourue = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("dstParcourue")));

                    if(!ssDstParcourue.empty())
                    {
		                xmlattr = pXMLDoc->createAttribute( XS("dstParcourue") );
		                xmlattr->setValue( XS(ssDstParcourue.c_str() ));
	                    xmlnodeVeh->setAttributeNode(xmlattr);
                    }

                    // Instant d'entrÃ©e
                    std::string ssInstEntree = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("instE")));

                    if (!ssInstEntree.empty())
                    {
                        xmlattr = pXMLDoc->createAttribute(XS("instE"));
                        xmlattr->setValue(XS(ssInstEntree.c_str()));
                        xmlnodeVeh->setAttributeNode(xmlattr);
                    }

                    // Itineraire
                    std::string ssItineraire = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("itineraire")));

                    if (!ssItineraire.empty())
                    {
                        xmlattr = pXMLDoc->createAttribute(XS("itineraire"));
                        xmlattr->setValue(XS(ssItineraire.c_str()));
                        xmlnodeVeh->setAttributeNode(xmlattr);
                    }

                    // rmq. : en toute rigueur, il manque les Ã©ventueles additional attributes...
				}
				else
				{
					// Importation de la totalitÃ© du noeud
					DOMNode* xmlnodeimp = pXMLDoc->importNode(xmlnodelistVehs->item(i), true);
					m_XmlNodeVehicules->appendChild(xmlnodeimp);

                    // Si le vÃ©hicule n'est pas sorti, on le met dans notre map d'accÃ¨s rapide pour Ã©viter les SelectSingleNodes !
                    std::string ssInstSortie = US(((DOMElement*)xmlnodelistVehs->item(i))->getAttribute(XS("instS")));
                    if (ssInstSortie.empty())
                    {
                        m_VehNodes[atoi(ssID.c_str())] = (DOMElement*)xmlnodeimp;
                    }
				}
			}
		}
	}

	delete XmlReader;
}

// Retourne le flux XML du dernier instant
DOMNode*	XMLDocTrafic::GetLastInstant(bool bFull, bool bTraceIteratif)
{
    DOMNode* xmlnode = NULL;

    if(bFull)               // Document complet
        xmlnode = pXMLDoc;
    else
    {            
		if(m_pTemporaryInstXMLDoc != NULL)
        {
            xmlnode = m_XmlNodeInstant;
        }
    }   

    return xmlnode;
}

// Retourne le flux XML de la collection des SYM_TRONCONS
DOMNode*	XMLDocTrafic::GetSymTronconsNode()
{
    DOMNode* xmlnode;

    xmlnode = m_pNetwork->GetXMLUtil()->SelectSingleNode("./SIMULATION/SYM_TRONCONS", pXMLDoc);
	
	return xmlnode;                    
  
}

DOMNode*XMLDocTrafic::GetSymVehiclesNode() const
{
    return m_XmlNodeVehicules; 
}

// Ajout d'une description d'une cellule de discrÃ©tisation
void XMLDocTrafic::AddCellule(int nID, const std::string & strLibelle, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav,  double dbYav, double dbZav)
{
	DOMElement* xmlnodeCell;
	DOMAttr* xmlattr;

	xmlnodeCell = pXMLDoc->createElement(XS("SGT"));

	// ID
	xmlattr = pXMLDoc->createAttribute( XS("id") );
	xmlattr->setValue( XS(SystemUtil::ToString(nID).c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(strLibelle.c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

	// Tuyau
	xmlattr = pXMLDoc->createAttribute( XS("tron") );
	xmlattr->setValue( XS(strTuyau.c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

	// X amont
	xmlattr = pXMLDoc->createAttribute( XS("Xam") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbXam).c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

	// X aval
	xmlattr = pXMLDoc->createAttribute( XS("Xav") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbXav).c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

	// Y amont
	xmlattr = pXMLDoc->createAttribute( XS("Yam") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbYam).c_str()) );
	xmlnodeCell->setAttributeNode(xmlattr);

	// Y aval
	xmlattr = pXMLDoc->createAttribute( XS("Yav") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbYav).c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

    // Z amont
	xmlattr = pXMLDoc->createAttribute( XS("Zam") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbZam).c_str()) );
	xmlnodeCell->setAttributeNode(xmlattr);

	// Z aval
	xmlattr = pXMLDoc->createAttribute( XS("Zav") );
	xmlattr->setValue( XS(SystemUtil::ToString(2, dbZav).c_str()) );
	xmlnodeCell->setAttributeNode(xmlattr);

	// Append		
	m_XmlNodeCellules->appendChild( xmlnodeCell );		
}

// Ajout d'une description d'un tronÃ§on (dÃ©fini par l'utilisateur ou construit par Symubruit)
void XMLDocTrafic::AddTroncon(const std::string & strLibelle, Point* pPtAm, Point* pPtAv, int nVoie , const std::string & ssBrique, std::deque<Point*> &lstPtInterne, double dbLong, std::vector<double> dbLarg, double dbStockMax, double dbStockInitial, Tuyau * pTuyau)
{
	DOMElement* xmlnodeCell;
    DOMNode* xmlnodePoints;
    DOMElement* xmlnodePoint;
	DOMAttr* xmlattr;

	int nPrecision = 2;
		if( m_pCoordTransf)
			if(m_pCoordTransf->GetTargetCS()->IsGeographic())
				nPrecision = 10;

	xmlnodeCell = pXMLDoc->createElement(XS("SYM_TRONCON"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(strLibelle.c_str()) );
	xmlnodeCell->setAttributeNode(xmlattr);
	
	// ExtremitÃ© amont 
	xmlattr = pXMLDoc->createAttribute( XS("extremite_amont") );
	xmlattr->setValue(XS((SystemUtil::ToString(nPrecision, pPtAm->dbX) + " " + SystemUtil::ToString(nPrecision, pPtAm->dbY) + " " + SystemUtil::ToString(nPrecision, pPtAm->dbZ)).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // ExtremitÃ© aval 
	xmlattr = pXMLDoc->createAttribute( XS("extremite_aval") );
	xmlattr->setValue(XS((SystemUtil::ToString(nPrecision, pPtAv->dbX) + " " + SystemUtil::ToString(nPrecision, pPtAv->dbY) + " " + SystemUtil::ToString(nPrecision, pPtAv->dbZ)).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // Longueur
    xmlattr = pXMLDoc->createAttribute( XS("longueur") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbLong).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // Largeur
    xmlattr = pXMLDoc->createAttribute( XS("largeur_voie") );
    std::string strLargeurs;
    for(size_t i = 0; i < dbLarg.size(); i++)
    {
        strLargeurs = strLargeurs.append(SystemUtil::ToString(2, dbLarg[i]));
        if(i != dbLarg.size()-1)
        {
            strLargeurs = strLargeurs.append(" ");
        }
    }
	xmlattr->setValue(XS(strLargeurs.c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);
  
    // Nombre de voie
    xmlattr = pXMLDoc->createAttribute( XS("nb_voie") );
	xmlattr->setValue(XS(SystemUtil::ToString(nVoie).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // Brique parente (si tronÃ§on virtuel)
    if(ssBrique.length()>0)
    {
        xmlattr = pXMLDoc->createAttribute( XS("brique") );
	    xmlattr->setValue( XS(ssBrique.c_str() ));
	    xmlnodeCell->setAttributeNode(xmlattr);
    }


    // Points internes
    if(lstPtInterne.size())
    {            
        xmlnodePoints = pXMLDoc->createElement(XS("POINTS_INTERNES"));
        xmlnodeCell->appendChild(xmlnodePoints);

        for(int i=0; i<(int)lstPtInterne.size(); i++)
        {
            xmlnodePoint = pXMLDoc->createElement(XS("POINT_INTERNE"));
            
	        xmlattr = pXMLDoc->createAttribute( XS("coordonnee") );
			xmlattr->setValue(XS((SystemUtil::ToString(nPrecision, lstPtInterne[i]->dbX) + " " + SystemUtil::ToString(nPrecision, lstPtInterne[i]->dbY) + " " + SystemUtil::ToString(nPrecision, lstPtInterne[i]->dbZ)).c_str()));

	        xmlnodePoint->setAttributeNode(xmlattr);
            xmlnodePoints->appendChild(xmlnodePoint);
        }
    }

    if(dbStockMax != UNDEF_STOCK)
    {
        xmlattr = pXMLDoc->createAttribute( XS("stock_max") );
        xmlattr->setValue(XS(SystemUtil::ToString(2, dbStockMax).c_str()));
	    xmlnodeCell->setAttributeNode(xmlattr);

        xmlattr = pXMLDoc->createAttribute( XS("stock_initial") );
        xmlattr->setValue(XS(SystemUtil::ToString(2, dbStockInitial).c_str()));
	    xmlnodeCell->setAttributeNode(xmlattr);
    }

	// Append		
	m_XmlNodeTroncons->appendChild( xmlnodeCell );		
}

// Ajout d'une description d'une voie (dÃ©fini par l'utilisateur ou construit par Symubruit)
void XMLDocTrafic::AddVoie(Point* pPtAm, Point* pPtAv, std::deque<Point*> &lstPtInterne, double dbLong)
{
	DOMElement* xmlnodeCell;
    DOMNode* xmlnodePoints;
    DOMElement* xmlnodePoint;
	DOMAttr* xmlattr;

	xmlnodeCell = pXMLDoc->createElement(XS("SYM_VOIE"));				
	
	// ExtremitÃ© amont 
	xmlattr = pXMLDoc->createAttribute( XS("extremite_amont") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, pPtAm->dbX) + " " + SystemUtil::ToString(2, pPtAm->dbY) + " " + SystemUtil::ToString(2, pPtAm->dbZ)).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // ExtremitÃ© aval 
	xmlattr = pXMLDoc->createAttribute( XS("extremite_aval") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, pPtAv->dbX) + " " + SystemUtil::ToString(2, pPtAv->dbY) + " " + SystemUtil::ToString(2, pPtAv->dbZ)).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);

    // Longueur
    xmlattr = pXMLDoc->createAttribute( XS("longueur") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbLong).c_str()));
	xmlnodeCell->setAttributeNode(xmlattr);
          
    // Points internes
    if(lstPtInterne.size())
    {            
        xmlnodePoints = pXMLDoc->createElement(XS("POINTS_INTERNES"));
        xmlnodeCell->appendChild(xmlnodePoints);

        for(int i=0; i<(int)lstPtInterne.size(); i++)
        {
            xmlnodePoint = pXMLDoc->createElement(XS("POINT_INTERNE"));
            
	        xmlattr = pXMLDoc->createAttribute( XS("coordonnee") );
			xmlattr->setValue(XS((SystemUtil::ToString(2, lstPtInterne[i]->dbX) + " " + SystemUtil::ToString(2, lstPtInterne[i]->dbY) + " " + SystemUtil::ToString(2, lstPtInterne[i]->dbZ)).c_str()));

	        xmlnodePoint->setAttributeNode(xmlattr);
            xmlnodePoints->appendChild(xmlnodePoint);
        }
    }

	// Append		
	m_XmlNodeTroncons->getLastChild()->appendChild( xmlnodeCell );		
}

// Ajout d'une description d'un capteur (de l'utilisateur ou crÃ©Ã© par Symubruit)
void XMLDocTrafic::AddDefCapteur(const std::string & sId, const std::string & sT, double dbPos, Point ptCoord )
{
	DOMElement* xmlnodeCpt;        
	DOMAttr* xmlattr;

	xmlnodeCpt = pXMLDoc->createElement(XS("SYM_CAPTEUR"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("id") );
	xmlattr->setValue( XS(sId.c_str() ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Type de capteur : longitudinal
	xmlattr = pXMLDoc->createAttribute( XS("type") );
	xmlattr->setValue( XS("ponctuel" ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Identifiant du tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("troncon") );
	xmlattr->setValue( XS(sT.c_str() ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Position sur le tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("position") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbPos).c_str()));
	xmlnodeCpt->setAttributeNode(xmlattr);

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoord.dbX) + " " + SystemUtil::ToString(2, ptCoord.dbY) + " " + SystemUtil::ToString(2, ptCoord.dbZ)).c_str()));
    xmlnodeCpt->setAttributeNode(xmlattr);;       

    // Append		
	m_XmlNodeSymCpts->appendChild( xmlnodeCpt );
}

// Ajout d'une description d'un capteur longitudinal (de l'utilisateur)
void XMLDocTrafic::AddDefCapteurLongitudinal(const std::string & sId, const std::string & sT, double dbPosDebut, Point ptCoordDebut, double dbPosFin, Point ptCoordFin)
{
	DOMElement* xmlnodeCpt;        
	DOMAttr* xmlattr;

	xmlnodeCpt = pXMLDoc->createElement(XS("SYM_CAPTEUR"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("id") );
	xmlattr->setValue( XS(sId.c_str() ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Type de capteur : longitudinal
	xmlattr = pXMLDoc->createAttribute( XS("type") );
	xmlattr->setValue( XS("longitudinal") ) ;
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Identifiant du tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("troncon") );
	xmlattr->setValue( XS(sT.c_str() )) ;
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Position sur le tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("position_debut") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbPosDebut).c_str()));
	xmlnodeCpt->setAttributeNode(xmlattr);

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees_debut") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoordDebut.dbX) + " " + SystemUtil::ToString(2, ptCoordDebut.dbY) + " " + SystemUtil::ToString(2, ptCoordDebut.dbZ)).c_str()));
    xmlnodeCpt->setAttributeNode(xmlattr);;        

    // Position sur le tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("position_fin") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbPosFin).c_str()));
	xmlnodeCpt->setAttributeNode(xmlattr);;

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees_fin") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoordFin.dbX) + " " + SystemUtil::ToString(2, ptCoordFin.dbY) + " " + SystemUtil::ToString(2, ptCoordFin.dbZ)).c_str()));
    xmlnodeCpt->setAttributeNode(xmlattr);;        

    // Append		
	m_XmlNodeSymCpts->appendChild( xmlnodeCpt );
}

// Ajout d'une description d'un capteur Edie (de l'utilisateur)
void XMLDocTrafic::AddDefCapteurEdie(const std::string & sId, const std::string & sT, double dbPosDebut, Point ptCoordDebut, double dbPosFin, Point ptCoordFin)
{
	DOMElement* xmlnodeCpt;        
	DOMAttr* xmlattr;

	xmlnodeCpt = pXMLDoc->createElement(XS("SYM_CAPTEUR"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("id") );
	xmlattr->setValue( XS(sId.c_str() ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Type de capteur : longitudinal
	xmlattr = pXMLDoc->createAttribute( XS("type") );
	xmlattr->setValue( XS("edie") ) ;
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Identifiant du tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("troncon") );
	xmlattr->setValue( XS(sT.c_str() ) );
	xmlnodeCpt->setAttributeNode(xmlattr);

    // Position sur le tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("position_debut") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbPosDebut).c_str()));
	xmlnodeCpt->setAttributeNode(xmlattr);

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees_debut") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoordDebut.dbX) + " " + SystemUtil::ToString(2, ptCoordDebut.dbY) + " " + SystemUtil::ToString(2, ptCoordDebut.dbZ)).c_str()));
    xmlnodeCpt->setAttributeNode(xmlattr);;        

    // Position sur le tronÃ§on
    xmlattr = pXMLDoc->createAttribute( XS("position_fin") );
	xmlattr->setValue(XS(SystemUtil::ToString(2, dbPosFin).c_str()));
	xmlnodeCpt->setAttributeNode(xmlattr);;

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees_fin") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoordFin.dbX) + " " + SystemUtil::ToString(2, ptCoordFin.dbY) + " " + SystemUtil::ToString(2, ptCoordFin.dbZ)).c_str()));
    xmlnodeCpt->setAttributeNode(xmlattr);;        

    // Append		
	m_XmlNodeSymCpts->appendChild( xmlnodeCpt );
}

// Ajout d'une description d'un capteur MFD (de l'utilisateur)
void XMLDocTrafic::AddDefCapteurMFD(const std::string & sId, bool bIncludeStrictData, const std::vector<Tuyau*>& lstTuyaux)
{
}

// Ajout d'une description d'une entrÃ©e
void XMLDocTrafic::AddDefEntree(const std::string & sId, Point ptCoord )
{
	DOMElement* xmlnodeE;        
	DOMAttr* xmlattr;

	xmlnodeE = pXMLDoc->createElement(XS("SYM_ENTREE"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(sId.c_str() ));
	xmlnodeE->setAttributeNode(xmlattr);       

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoord.dbX) + " " + SystemUtil::ToString(2, ptCoord.dbY) + " " + SystemUtil::ToString(2, ptCoord.dbZ)).c_str()));
    xmlnodeE->setAttributeNode(xmlattr);       

    // Append		
	m_XmlNodeSymEntrees->appendChild( xmlnodeE );
}

// Ajout d'une description d'une sortie
void XMLDocTrafic::AddDefSortie(const std::string & sId, Point ptCoord )
{
	DOMElement* xmlnodeS;        
	DOMAttr* xmlattr;

	xmlnodeS = pXMLDoc->createElement(XS("SYM_SORTIE"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(sId.c_str() ));
	xmlnodeS->setAttributeNode(xmlattr);       

    // CoordonnÃ©es
    xmlattr = pXMLDoc->createAttribute( XS("coordonnees") );
	xmlattr->setValue(XS((SystemUtil::ToString(2, ptCoord.dbX) + " " + SystemUtil::ToString(2, ptCoord.dbY) + " " + SystemUtil::ToString(2, ptCoord.dbZ)).c_str()));
    xmlnodeS->setAttributeNode(xmlattr);        

    // Append		
	m_XmlNodeSymSorties->appendChild( xmlnodeS );
}

// Ajout d'une description d'un CAF
void XMLDocTrafic::AddDefCAF(CarrefourAFeuxEx * pCAF)
{

}

// Ajout d'une description d'un Giratoire
void XMLDocTrafic::AddDefGir(Giratoire * pGir)
{

}

// Ajout d'une description d'un CDF
void XMLDocTrafic::AddDefCDF(ControleurDeFeux * pCDF)
{

}

// Ajout d'une description d'un arrÃªt de bus
void XMLDocTrafic::AddArret(const std::string & strLibelle, int numVoie, char * strNomLigne)
{
	DOMElement* xmlnodeCell;
	DOMAttr* xmlattr; 

	xmlnodeCell = pXMLDoc->createElement(XS("ARRET"));
	
	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(strLibelle.c_str() ));
	xmlnodeCell->setAttributeNode(xmlattr);

    // CoordonnÃ©es
 //   xmlattr = pXMLDoc->createAttribute( "coordonnees" );
	//xmlattr->setValue((SystemUtil::ToString(2, dbX) + " " + SystemUtil::ToString(2, dbY) + " " + SystemUtil::ToString(2, dbZ)).c_str());
 //   xmlnodeCell->setAttributeNode(xmlattr);   

    // identifiant de la ligne
    xmlattr = pXMLDoc->createAttribute( XS("ligne") );
    xmlattr->setValue(XS(strNomLigne));
    xmlnodeCell->setAttributeNode(xmlattr);   

    // numÃ©ro de la voie
    xmlattr = pXMLDoc->createAttribute( XS("num_voie") );
    xmlattr->setValue(XS(SystemUtil::ToString(numVoie).c_str()));
    xmlnodeCell->setAttributeNode(xmlattr); 
		
	// Append		
	m_XmlNodeArrets->appendChild( xmlnodeCell );		
}

// Ajout de la description d'un vÃ©hicule
void XMLDocTrafic::AddVehicule(int nID, const std::string & strLibelle, const std::string & strType, const std::string & strGMLType, double dbKx, double dbVx, double dbW, const std::string & strEntree, const std::string & strSortie, const std::string & strZoneEntree, const std::string & strZoneSortie, const std::string & strRoute, double dbInstCreation, const std::string & sVoie, bool bAgressif, int iLane, const std::string & sLine, const std::vector<std::string> & initialPath, const std::deque<PlageAcceleration> & plagesAcc, const std::string & motifOrigine, const std::string & motifDestination, bool bAddToCreationNode)
{
	DOMElement* xmlnodeVeh;
	DOMAttr* xmlattr;

	xmlnodeVeh = pXMLDoc->createElement(XS("VEH"));

	// ID
	xmlattr = pXMLDoc->createAttribute( XS("id") );
	xmlattr->setValue( XS(SystemUtil::ToString(nID).c_str() ));
	xmlnodeVeh->setAttributeNode(xmlattr);

	// Libelle
	xmlattr = pXMLDoc->createAttribute( XS("lib") );
	xmlattr->setValue( XS(strLibelle.c_str() ));
	xmlnodeVeh->setAttributeNode(xmlattr);
	
	// Type de vÃ©hicule
	xmlattr = pXMLDoc->createAttribute( XS("type") );
	xmlattr->setValue( XS(strType.c_str() ));
	xmlnodeVeh->setAttributeNode(xmlattr);	

    // Kx
    xmlattr = pXMLDoc->createAttribute( XS("kx") );		
    xmlattr->setValue(XS(SystemUtil::ToString(3, dbKx).c_str()));
	xmlnodeVeh->setAttributeNode(xmlattr);

    // Vx
    xmlattr = pXMLDoc->createAttribute( XS("vx") );		
	xmlattr->setValue(XS(SystemUtil::ToString(3, dbVx).c_str()));
	xmlnodeVeh->setAttributeNode(xmlattr);

    // W
    xmlattr = pXMLDoc->createAttribute( XS("w") );		
	xmlattr->setValue(XS(SystemUtil::ToString(3, dbW).c_str()));
	xmlnodeVeh->setAttributeNode(xmlattr);

    // EntrÃ©e
	xmlattr = pXMLDoc->createAttribute( XS("entree") );
	xmlattr->setValue( XS(strEntree.c_str() ));
	xmlnodeVeh->setAttributeNode(xmlattr);

	// Instant de crÃ©ation
    xmlattr = pXMLDoc->createAttribute( XS("instC") );		
	xmlattr->setValue(XS(SystemUtil::ToString(3, dbInstCreation).c_str()));
	xmlnodeVeh->setAttributeNode(xmlattr);    

    // Append		
	m_XmlNodeVehicules->appendChild( xmlnodeVeh );	

	// Agressif ?
	xmlattr = pXMLDoc->createAttribute( XS("agressif") );		
	xmlattr->setValue(XS(SystemUtil::ToString(bAgressif).c_str()));
	xmlnodeVeh->setAttributeNode(xmlattr);

    // Motifs
    if (!motifOrigine.empty())
    {
        xmlattr = pXMLDoc->createAttribute(XS("motif_origine"));
        xmlattr->setValue(XS(motifOrigine.c_str()));
        xmlnodeVeh->setAttributeNode(xmlattr);
    }
    if (!motifDestination.empty())
    {
        xmlattr = pXMLDoc->createAttribute(XS("motif_destination"));
        xmlattr->setValue(XS(motifDestination.c_str()));
        xmlnodeVeh->setAttributeNode(xmlattr);
    }

    // Zones
    if (!strZoneEntree.empty())
    {
        xmlattr = pXMLDoc->createAttribute(XS("zone_entree"));
        xmlattr->setValue(XS(strZoneEntree.c_str()));
        xmlnodeVeh->setAttributeNode(xmlattr);
    }
    if (!strZoneSortie.empty())
    {
        xmlattr = pXMLDoc->createAttribute(XS("zone_sortie"));
        xmlattr->setValue(XS(strZoneSortie.c_str()));
        xmlnodeVeh->setAttributeNode(xmlattr);
    }
    

    // MAJ de la map des noeuds des vÃ©hicules
    m_VehNodes[nID] = xmlnodeVeh;

    if(m_XmlNodeCreations && bAddToCreationNode)
    {
       AddVehiculeToCreationNode( nID, strLibelle,  strType,  dbKx,  dbVx,  dbW, strEntree, strSortie,  strRoute,  dbInstCreation,  sVoie,  bAgressif);
    }
}

// ajout la description d'un vÃ©hicule sur sa crÃ©ation
void XMLDocTrafic::AddVehiculeToCreationNode(int nID, const std::string & strLibelle, const std::string & strType, double dbKx, double dbVx, double dbW, const std::string & strEntree, const std::string & strSortie, const std::string & strRoute, double dbInstCreation, const std::string & sVoie, bool bAgressif) 
{
    DOMElement* xmlnodeVeh;
    xmlnodeVeh = m_XmlNodeCreations->getOwnerDocument()->createElement(XS("CREATION"));

	// ID
	xmlnodeVeh->setAttribute(XS("id"), XS(SystemUtil::ToString(nID).c_str()));	

    // type
	xmlnodeVeh->setAttribute(XS("type"), XS(strType.c_str()));

    // entree
	xmlnodeVeh->setAttribute(XS("entree"), XS(strEntree.c_str()));

    // sortie
	xmlnodeVeh->setAttribute(XS("sortie"), XS(strSortie.c_str()));

    // route
    if(!strRoute.empty())
    {
	    xmlnodeVeh->setAttribute(XS("route"), XS(strRoute.c_str()));
    }

    // Append						
	m_XmlNodeCreations->appendChild( xmlnodeVeh );	
}

// ajout la description d'un vÃ©hicule lors de sa sortie du rÃ©seau
void XMLDocTrafic::AddVehiculeToSortieNode(int nID, const std::string & strSortie, double dbInstSortie, double dbVit)
{
	DOMElement* xmlnodeVeh;
	xmlnodeVeh = m_XmlNodeSorties->getOwnerDocument()->createElement(XS("SORTIE"));

	// ID
	xmlnodeVeh->setAttribute(XS("id"), XS(SystemUtil::ToString(nID).c_str()));

	// sortie
	xmlnodeVeh->setAttribute(XS("sortie"), XS(strSortie.c_str()));

	// instant de sortie
	xmlnodeVeh->setAttribute(XS("instant"), XS(SystemUtil::ToString(2, dbInstSortie).c_str()));

	// vitesse Ã  l'instant de sortie
	xmlnodeVeh->setAttribute(XS("instant"), XS(SystemUtil::ToString(2, dbVit).c_str()));

	// Append						
	m_XmlNodeSorties->appendChild(xmlnodeVeh);
}


// Ajout des sorties d'une brique de rÃ©gulation
void XMLDocTrafic::AddSimuRegulation(DOMNode * pRestitutionNode)
{
    m_XmlNodeRegulations->appendChild(pRestitutionNode);
}

// Ajout de la description des donnÃ©es trafic d'une entrÃ©e
void XMLDocTrafic::AddSimuEntree(const std::string & sEntree, int nVeh)
{
	DOMElement* xmlnodeEntree;

    xmlnodeEntree = m_XmlNodeEntrees->getOwnerDocument()->createElement(XS("ENTREE"));

	// ID de l'entrÃ©e
	xmlnodeEntree->setAttribute(XS("id"), XS(sEntree.c_str()));

	// Nombre de vÃ©hicules
	xmlnodeEntree->setAttribute(XS("nb_veh_en_attente"), XS(SystemUtil::ToString(nVeh).c_str()));				

	m_XmlNodeEntrees->appendChild( xmlnodeEntree );
}


// Ajout de la description des donnÃ©es trafic d'un parking
void XMLDocTrafic::AddSimuParking(const std::string & sParking, int nStock, int nVehAttente)
{
	DOMElement* xmlnodeParking;

    if(m_bTraceStocks)
    {
        xmlnodeParking = m_XmlNodeParkings->getOwnerDocument()->createElement(XS("PARKING"));

	    // ID de l'entrÃ©e
	    xmlnodeParking->setAttribute(XS("id"), XS(sParking.c_str()));

        // Nombre de vÃ©hicules
	    xmlnodeParking->setAttribute(XS("stock"), XS(SystemUtil::ToString(nStock).c_str()));			

	    // Nombre de vÃ©hicules
	    xmlnodeParking->setAttribute(XS("nb_veh_en_attente"), XS(SystemUtil::ToString(nVehAttente).c_str()));				

	    m_XmlNodeParkings->appendChild( xmlnodeParking );
    }
}

// Ajout de la description des donnÃ©es trafic d'un troncon de stationnement
void XMLDocTrafic::AddSimuTronconStationnement(const std::string & sTroncon, double dbLongueur)
{
	DOMElement* xmlnodeTroncon;

    if(m_bTraceStocks)
    {
        xmlnodeTroncon = m_XmlNodeTronconsStationnement->getOwnerDocument()->createElement(XS("TRONCON_STATIONNEMENT"));

	    // ID du tronÃ§on
	    xmlnodeTroncon->setAttribute(XS("id"), XS(sTroncon.c_str()));

        // longueur de stationnement
	    xmlnodeTroncon->setAttribute(XS("stock"), XS(SystemUtil::ToString(2, dbLongueur).c_str()));			

        m_XmlNodeTronconsStationnement->appendChild( xmlnodeTroncon );
    }
}



// Mise Ã  jour de l'instant de sortie du vÃ©hicule
void XMLDocTrafic::UpdateInstSortieVehicule(int nID, double dbInstSortie, const std::string & sSortie, double dstParcourue,
    const std::vector<Tuyau*> & itinerary, const std::map<std::string, std::string> & additionalAttributes)
{
    DOMElement*        xmlnodeVeh;

    xmlnodeVeh = m_VehNodes[nID];
	if(!xmlnodeVeh)
	{
        std::string ssPath = "./VEH[@id=\"" + SystemUtil::ToString(nID) + "\"]";
        xmlnodeVeh = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode(ssPath, m_XmlNodeVehicules->getOwnerDocument(), m_XmlNodeVehicules);

        if(!xmlnodeVeh)
        {
		    this->AddVehicule(nID, "", "", "", 0, 0, 0, "", "", "", "", "", 0, "", false, 0, "", std::vector<std::string>(), std::deque<PlageAcceleration>(), "", "", false);
            xmlnodeVeh = m_VehNodes[nID];
        }
        else
        {
            m_VehNodes[nID] = xmlnodeVeh;
        }
	}

    if(xmlnodeVeh)
    {
        // Sortie
        xmlnodeVeh->setAttribute(XS("sortie"), XS(sSortie.c_str()));

        // Instant de sortie
	    xmlnodeVeh->setAttribute(XS("instS"), XS(SystemUtil::ToString(2, dbInstSortie).c_str()));

		// Distance cumulÃ©e
	    xmlnodeVeh->setAttribute(XS("dstParcourue"), XS(SystemUtil::ToString(2, dstParcourue).c_str()));

        // Itineraire parcouru
        std::string strItinerary;
        Tuyau * pLink;
        for (size_t iLink = 0; iLink < itinerary.size(); iLink++)
        {
            pLink = itinerary.at(iLink);
            if (!pLink->GetBriqueParente())
            {
                if (!strItinerary.empty())
                {
                    strItinerary.append(" ");
                }
                strItinerary.append(pLink->GetLabel());
            }
        }

        if (!strItinerary.empty())
        {
            xmlnodeVeh->setAttribute(XS("itineraire"), XS(strItinerary.c_str()));
        }

        // Ajout des infos supplÃ©mentaires le cas Ã©chÃ©ant
        std::map<std::string, std::string>::const_iterator iterAttr;
        for(iterAttr = additionalAttributes.begin(); iterAttr != additionalAttributes.end(); ++iterAttr)
        {
            xmlnodeVeh->setAttribute(XS(iterAttr->first.c_str()), XS(iterAttr->second.c_str()));
        }
    }	

    // Nettoyage de la map (normalement on ne touche plus au noeud une fois le vÃ©hicule sorti)
    m_VehNodes.erase(nID);
}

// Mise Ã  jour de l'instant d'entrÃ©e du vÃ©hicule
void XMLDocTrafic::UpdateInstEntreeVehicule(int nID, double dbInstEntree)
{
    DOMElement*        xmlnodeVeh = NULL;

    xmlnodeVeh = m_VehNodes[nID];
	if(!xmlnodeVeh)
	{
        std::string ssPath = "./VEH[@id=\"" + SystemUtil::ToString(nID) + "\"]";
        xmlnodeVeh = (DOMElement*)m_pNetwork->GetXMLUtil()->SelectSingleNode(ssPath, m_XmlNodeVehicules->getOwnerDocument(), m_XmlNodeVehicules);

        if(!xmlnodeVeh)
	    {
            this->AddVehicule(nID, "", "", "", 0, 0, 0, "", "", "", "", "", 0, "", false, 0, "", std::vector<std::string>(), std::deque<PlageAcceleration>(), "", "", false);
            xmlnodeVeh = m_VehNodes[nID];
        }
        else
        {
            m_VehNodes[nID] = xmlnodeVeh;
        }
	}

    if(xmlnodeVeh)
    {        
        // Instant d'entrÃ©e
	    xmlnodeVeh->setAttribute(XS("instE"), XS(SystemUtil::ToString(3, dbInstEntree).c_str()));
    }	
}

// Suppression des vÃ©hicules mÃ©morisÃ©es Ã  partir d'un certain ID
void XMLDocTrafic::RemoveVehicules(int nFromID)
{
	DOMXPathResult*			xmlnodelistVeh;
    std::string ssPath;        

	ssPath = "VEH[@id >= " + SystemUtil::ToString(nFromID) + "]";	

    xmlnodelistVeh = m_pNetwork->GetXMLUtil()->SelectNodes(ssPath, m_XmlNodeVehicules->getOwnerDocument(), m_XmlNodeVehicules);
	for (XMLSize_t i=0; i<xmlnodelistVeh->getSnapshotLength(); i++)
	{
		xmlnodelistVeh->snapshotItem(i);
		DOMNode* xmlChild = m_XmlNodeVehicules->removeChild( xmlnodelistVeh->getNodeValue() );
        m_pNetwork->GetXMLUtil()->ReleaseNodes(xmlChild);
	}

	xmlnodelistVeh->release();

    // nettoyage de la map
    std::map<int, DOMElement*>::iterator it = m_VehNodes.begin();
    std::map<int, DOMElement*>::iterator itEnd = m_VehNodes.end();
    while(it != itEnd)
    {
        if((*it).first >= nFromID)
        {
            m_VehNodes.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}

// Ajout de l'Ã©tat des feux pour l'instant considÃ©rÃ©
// dÃ©finition au prÃ©alable des noms de attributs sous forme de XMLCh[] pour Ã©viter des transcode nombreux inutiles
const XMLCh  gXMLDecl_FEU[] =
{
    chLatin_F, chLatin_E, chLatin_U, chNull
};
const XMLCh  gXMLDecl_ctrl_feux[] =
{
    chLatin_c, chLatin_t, chLatin_r, chLatin_l, chUnderscore, chLatin_f, chLatin_e, chLatin_u, chLatin_x, chNull
};
const XMLCh  gXMLDecl_troncon_entree[] =
{
    chLatin_t, chLatin_r, chLatin_o, chLatin_n, chLatin_c, chLatin_o, chLatin_n, chUnderscore, chLatin_e, chLatin_n, chLatin_t, chLatin_r, chLatin_e, chLatin_e, chNull
};
const XMLCh  gXMLDecl_troncon_sortie[] =
{
    chLatin_t, chLatin_r, chLatin_o, chLatin_n, chLatin_c, chLatin_o, chLatin_n, chUnderscore, chLatin_s, chLatin_o, chLatin_r, chLatin_t, chLatin_i, chLatin_e, chNull
};
const XMLCh  gXMLDecl_etat[] =
{
    chLatin_e, chLatin_t, chLatin_a, chLatin_t, chNull
};
const XMLCh  gXMLDecl_debut_cycle[] =
{
    chLatin_d, chLatin_e, chLatin_b, chLatin_u, chLatin_t, chUnderscore, chLatin_c, chLatin_y, chLatin_c, chLatin_l, chLatin_e, chNull
};
const XMLCh  gXMLDecl_prioritaire[] =
{
    chLatin_p, chLatin_r, chLatin_i, chLatin_o, chLatin_r, chLatin_i, chLatin_t, chLatin_a, chLatin_i, chLatin_r, chLatin_e, chNull
};
void XMLDocTrafic::AddSimFeux(const std::string & sCtrlFeux, const std::string & sTE, const std::string & sTS, int bEtatFeu, int bPremierInstCycle, int bPrioritaire)
{
    if(m_bSave)
    {
        DOMElement* xmlnodeFeu;        

        xmlnodeFeu = m_XmlNodeFeux->getOwnerDocument()->createElement(gXMLDecl_FEU);
        // Ajout		
	    m_XmlNodeFeux->appendChild( xmlnodeFeu );

        // Controleur de feux
        xmlnodeFeu->setAttribute(gXMLDecl_ctrl_feux, XS(sCtrlFeux.c_str()));	

        // TronÃ§on d'entrÃ©e
        xmlnodeFeu->setAttribute(gXMLDecl_troncon_entree, XS(sTE.c_str()));	

        // TronÃ§on de sortie
        xmlnodeFeu->setAttribute(gXMLDecl_troncon_sortie, XS(sTS.c_str()));	

        // attribut Ã©tat
        xmlnodeFeu->setAttribute(gXMLDecl_etat, XS(SystemUtil::ToString(bEtatFeu).c_str()));	       

	    // Attribut premier instant du cycle de feux
        xmlnodeFeu->setAttribute(gXMLDecl_debut_cycle, XS(SystemUtil::ToString(bPremierInstCycle).c_str()));	 

        // Attribut sÃ©quence liÃ©e Ã  un vÃ©hicule guidÃ© prioritaire
        xmlnodeFeu->setAttribute(gXMLDecl_prioritaire, XS(SystemUtil::ToString(bPrioritaire).c_str()));
    }
}

void XMLDocTrafic::AddSimFeuxEVE(const std::string & sCtrlFeux, const std::string & sTE, const std::string & sTS, int bEtatFeu, int bPremierInstCycle, int bPrioritaire)
{

}

// Ajout des donnÃ©es complÃ¨te trafic d'une cellule de discrÃ©tisation pour l'instant considÃ©rÃ©
void XMLDocTrafic::AddCellSimu(int nID, double dbConc, double dbDebit, double dbVitAm, double dbAccAm, double dbNAm, double dbVitAv, double dbAccAv, double dbNAv,
	const std::string & strLibelle, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav,  double dbYav, double dbZav)
{
	DOMElement* xmlnodeCell;

    xmlnodeCell = m_XmlNodeCellules->getOwnerDocument()->createElement(XS("SGT"));

	// ID
	xmlnodeCell->setAttribute(XS("id"), XS(SystemUtil::ToString(nID).c_str()));	

	// Concentration
	xmlnodeCell->setAttribute(XS("Conc"), XS(SystemUtil::ToString(2, dbConc).c_str()));	

	// DÃ©bit
	xmlnodeCell->setAttribute(XS("Deb"), XS(SystemUtil::ToString(2, dbDebit).c_str()));

	// Vitesse amont
	xmlnodeCell->setAttribute(XS("VitAm"), XS(SystemUtil::ToString(2, dbVitAm).c_str()));	

	// AccÃ©lÃ©ration amont
	xmlnodeCell->setAttribute(XS("AccAm"), XS(SystemUtil::ToString(2, dbAccAm).c_str()));	

	// N amont
	xmlnodeCell->setAttribute(XS("NAm"), XS(SystemUtil::ToString(2, dbNAm).c_str()));

	// Vitesse aval
	xmlnodeCell->setAttribute(XS("VitAv"), XS(SystemUtil::ToString(2, dbVitAv).c_str()));	

	// AccÃ©lÃ©ration aval
	xmlnodeCell->setAttribute(XS("AccAv"), XS(SystemUtil::ToString(2, dbAccAv).c_str()));	

	// N aval
	xmlnodeCell->setAttribute(XS("NAv"), XS(SystemUtil::ToString(2, dbNAv).c_str()));

	// Append		
    m_XmlNodeCellules->appendChild( xmlnodeCell ); 

    // OTK - rmq. la mÃ©thode AddCellSimu avec ces attributs en plus n'Ã©tait sauf erreur jamais appelÃ©e.
    // on conserve le code d'ajout des attributs supplÃ©mentaires ci-dessous au cas oÃ¹ il faille un jour les sortir.
    /*
    // Libelle
	xmlnodeCell->setAttribute("lib", strLibelle.c_str());

	// Tuyau
	xmlnodeCell->setAttribute("tron", strLibelle.c_str());

	// X amont
	xmlnodeCell->setAttribute("Xam", SystemUtil::ToString(2, dbXam).c_str());

	// X aval
	xmlnodeCell->setAttribute("Xav", SystemUtil::ToString(2, dbXav).c_str());

	// Y amont
	xmlnodeCell->setAttribute("Yam", SystemUtil::ToString(2, dbYam).c_str());

	// Y aval
	xmlnodeCell->setAttribute("Yav", SystemUtil::ToString(2, dbYav).c_str());

    // Z amont
	xmlnodeCell->setAttribute("Zam", SystemUtil::ToString(2, dbZam).c_str());

	// Z aval
	xmlnodeCell->setAttribute("Zav", SystemUtil::ToString(2, dbZav).c_str());
    */
}

// Ajout des donnÃ©es de la trajectoire d'un vÃ©hicule Ã  l'instant considÃ©rÃ©
void XMLDocTrafic::AddTrajectoire(int nID, Tuyau * pTuyau, const std::string & strTuyau, const std::string & strTuyauEx, const std::string & strNextTuyauEx, int nNumVoie, double dbAbs, double dbOrd, double dbZ, double dbAbsCur, double dbVit, double dbAcc, double dbDeltaN, const std::string & sTypeVehicule, double dbVitMax, double dbLongueur, const std::string & sLib, int nIDVehLeader, int nCurrentLoad,
    bool bTypeChgtVoie, TypeChgtVoie eTypeChgtVoie, bool bVoieCible, int nVoieCible, bool bPi, double dbPi, bool bPhi, double dbPhi, bool bRand, double dbRand,
    bool bDriven, const std::string & strDriveState, bool bDepassement, bool bRegimeFluide, const std::map<std::string, std::string> & additionalAttributes)
{
	DOMElement* xmlnodeVeh;
	DOMAttr* xmlattr;

    xmlnodeVeh = m_XmlNodetraj->getOwnerDocument()->createElement(XS("TRAJ"));

	// ID
	xmlnodeVeh->setAttribute(XS("id"), XS(SystemUtil::ToString(nID).c_str()));		
    
    // LibellÃ© du tuyau
    xmlnodeVeh->setAttribute(XS("tron"), XS(strTuyau.c_str()));

    //if(m_bSave) 	// ??? (29/06/2012) Correction BUG ID 6
    {
        // NumÃ©ro de voie (Ã  partir de 1)
        xmlnodeVeh->setAttribute(XS("voie"), XS(SystemUtil::ToString(nNumVoie).c_str()));
    }

    // Abscisse curviligne
    xmlnodeVeh->setAttribute(XS("dst"), XS(SystemUtil::ToString(2, dbAbsCur).c_str()));        

    //if(m_bSave)	// ??? (23/08/2010)
    {
		int nPrecision = 2;
		if( m_pCoordTransf)
			if(m_pCoordTransf->GetTargetCS()->IsGeographic())
				nPrecision = 10;

	    // Abscisse
	    xmlnodeVeh->setAttribute(XS("abs"), XS(SystemUtil::ToString(nPrecision, dbAbs).c_str()));

	    // OrdonnÃ©e
	    xmlnodeVeh->setAttribute(XS("ord"), XS(SystemUtil::ToString(nPrecision, dbOrd).c_str()));

        // Z
	    xmlnodeVeh->setAttribute(XS("z"), XS(SystemUtil::ToString(2, dbZ).c_str()));
    }

    if( !m_bSortieLight )
    {
	    // Vitesse
	    xmlnodeVeh->setAttribute(XS("vit"), XS(SystemUtil::ToString(2, dbVit).c_str()));

	    // AccÃ©lÃ©ration
	    xmlnodeVeh->setAttribute(XS("acc"), XS(SystemUtil::ToString(2, dbAcc).c_str()));
    }

    // Delta N
    if(m_bDebug)
    {
	    xmlnodeVeh->setAttribute(XS("deltaN"), XS(SystemUtil::ToString(2, dbDeltaN).c_str()));
    }

    // Type du vÃ©hicule
    if(sTypeVehicule.length() >0 )
    {
	    xmlnodeVeh->setAttribute(XS("type"), XS(sTypeVehicule.c_str()));
    }

    // LibellÃ© (uniquement pour les bus)
    if(sLib.length() >0 )
    {
	    xmlnodeVeh->setAttribute(XS("lib"), XS(sLib.c_str()));
    }

    // VÃ©hicule leader
    if(m_bDebug)
    {
	    xmlnodeVeh->setAttribute(XS("lead"), XS(SystemUtil::ToString(nIDVehLeader).c_str()));		
    }

    // attributs spÃ©cifiques aux changements de voie :
    if(bTypeChgtVoie)
    {
        xmlattr = xmlnodeVeh->getOwnerDocument()->createAttribute( XS("type_chgt_voie") );
        if(eTypeChgtVoie == eConfort)
        {
            xmlattr->setValue( XS("confort") );
        }
        else if(eTypeChgtVoie == eRabattement)
        {
            xmlattr->setValue( XS("rabattement") );
        }
        else
        {
            xmlattr->setValue( XS("force") );
        }
	    xmlnodeVeh->setAttributeNode(xmlattr);		
    }

    if(bVoieCible)
    {
	    xmlnodeVeh->setAttribute(XS("voie_cible"), XS(SystemUtil::ToString(nVoieCible).c_str()));	
    }

    if(bPi)
    {
	    xmlnodeVeh->setAttribute(XS("pi"), XS(SystemUtil::ToString(3, dbPi).c_str()));	
    }

    if(bPhi)
    {
	    xmlnodeVeh->setAttribute(XS("phi"), XS(SystemUtil::ToString(3, dbPhi).c_str()));	
    }

    if(bPhi)
    {
	    xmlnodeVeh->setAttribute(XS("phi"), XS(SystemUtil::ToString(3, dbPhi).c_str()));	
    }
    
    if(bRand)
    {
	    xmlnodeVeh->setAttribute(XS("rand"), XS(SystemUtil::ToString(3, dbRand).c_str()));	
    }

    // Ajout de l'attribut de pilotage
    if(bDriven)
    {
	    xmlnodeVeh->setAttribute(XS("etat_pilotage"), XS(strDriveState.c_str()));
    }

    // Ajout de l'attribut de dÃ©passement
    if(bDepassement)
    {
	    xmlnodeVeh->setAttribute(XS("depassement"), XS("true"));
    }

    if(m_bSortieRegime)
    {
        xmlnodeVeh->setAttribute(XS("free"), XS(bRegimeFluide ? "1" : "0"));
    }

    if(nCurrentLoad != -1)
    {
        xmlnodeVeh->setAttribute(XS("charge"), XS(SystemUtil::ToString(nCurrentLoad).c_str()));
    }

    // Ajout des infos supplÃ©mentaires le cas Ã©chÃ©ant
    std::map<std::string, std::string>::const_iterator iterAttr;
    for(iterAttr = additionalAttributes.begin(); iterAttr != additionalAttributes.end(); ++iterAttr)
    {
        xmlnodeVeh->setAttribute(XS(iterAttr->first.c_str()), XS(iterAttr->second.c_str()));
    }

	// Append						
	m_XmlNodetraj->appendChild( xmlnodeVeh );	
};


void XMLDocTrafic::AddStream(int nID, const std::string & strTuyau, const std::string & strTuyauEx)
{
    DOMElement* xmlnodeVeh;
    xmlnodeVeh = m_XmlNodeStream->getOwnerDocument()->createElement(XS("STREAM"));
    // ID
	xmlnodeVeh->setAttribute(XS("id"), XS(SystemUtil::ToString(nID).c_str()));	
      
    // LibellÃ© du tuyau
    xmlnodeVeh->setAttribute(XS("tron"), XS(strTuyau.c_str()));

    // Append						
	m_XmlNodeStream->appendChild( xmlnodeVeh );
}

void XMLDocTrafic::AddLink(const std::string &  strTuyau, double dbConcentration)
{
    DOMElement* xmlnodeVeh;
    xmlnodeVeh = m_XmlNodeLinks->getOwnerDocument()->createElement(XS("LINK"));
    
    // LibellÃ© du tuyau
	xmlnodeVeh->setAttribute(XS("id"), XS(strTuyau.c_str()));

    // Concentration des vÃ©hicules sur le tronÃ§on
    xmlnodeVeh->setAttribute(XS("concentration"), XS(SystemUtil::ToString(3, dbConcentration).c_str()));

    // Append						
	m_XmlNodeLinks->appendChild( xmlnodeVeh );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void XMLDocTrafic::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void XMLDocTrafic::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void XMLDocTrafic::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(DocTrafic);

    // membres "classiques"
    ar & BOOST_SERIALIZATION_NVP(m_strFilename);
    ar & BOOST_SERIALIZATION_NVP(m_bSave);
    ar & BOOST_SERIALIZATION_NVP(m_bDebug);
    ar & BOOST_SERIALIZATION_NVP(m_bTraceStocks);
    ar & BOOST_SERIALIZATION_NVP(m_bSortieLight);
    ar & BOOST_SERIALIZATION_NVP(m_bSortieRegime);
    ar & BOOST_SERIALIZATION_NVP(m_strTraceFile);
    ar & BOOST_SERIALIZATION_NVP(m_bAtLeastOneInstant);
    ar & BOOST_SERIALIZATION_NVP(m_pNetwork);

    // structure DOM
    SerialiseDOMDocument<Archive>(ar, "pXMLDoc", XS("OUT"), pXMLDoc, m_pNetwork->GetXMLUtil());

    std::string resultStr;
    if(Archive::is_saving::value)
    {
        // fichier rÃ©sultat : on ferme le fichier actuel et on rÃ©cupÃ¨re son contenu pour le sauvegarder dans 
        // le fichier de sÃ©rialisation
        Suspend();
        std::ifstream ifs(m_strFilename.c_str(), std::ios::in | std::ios::binary); // lecture en binaire sinon les CR LF deviennent LF
        std::stringstream ss;
        ss << ifs.rdbuf();
        ifs.close();
        resultStr = ss.str();
    }
    ar & BOOST_SERIALIZATION_NVP(resultStr);


    // on rÃ©initialise les autres membres dÃ©ductibles des autres et on rouvre le fichier de rÃ©sultat
    if(Archive::is_loading::value)
    {
        ReInit(resultStr);
    }
}