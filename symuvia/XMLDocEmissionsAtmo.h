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
#ifndef XMLDocEmissionAtmoH
#define XMLDocEmissionAtmoH

#include <xercesc/util/XercesVersion.hpp>

#include <string>

class DOMLSSerializerSymu;

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
    class DOMElement;
    class DOMDocument;
};
class STime;
class Reseau;

/*===========================================================================================*/
/* Classe de modélisation du document XML des émissions atmosphériques                       */
/*===========================================================================================*/
class XMLDocEmissionsAtmo
{
private:

	std::string				            m_strFilename;
	DOMLSSerializerSymu		            * m_XmlWriter;

	XERCES_CPP_NAMESPACE::DOMNode		* m_XmlNodeInstants;
	XERCES_CPP_NAMESPACE::DOMNode		* m_XmlNodeSimulation;
	XERCES_CPP_NAMESPACE::DOMElement	* m_XmlNodeVehicules;

	// Le document XML
	XERCES_CPP_NAMESPACE::DOMDocument   * pXMLDoc;

    Reseau                              * m_pNetwork;

public:
    // Constructeurs
	XMLDocEmissionsAtmo(Reseau * pNetwork, XERCES_CPP_NAMESPACE::DOMDocument* pXMLDocument)
	{
        m_pNetwork = pNetwork;
		pXMLDoc = pXMLDocument;
	}
	~XMLDocEmissionsAtmo();

    // Initialisation
	void Init(const std::string & strFilename, const STime &dtDebut, const std::string & strVersionExe, const std::string & strTimeTrafic, const std::string & strVersionDB, const std::string & strTitre, const std::string & strDate);

    // Fin
	void Terminate();

    // Retourne le flux XML du dernier instant
	XERCES_CPP_NAMESPACE::DOMNode *	GetLastInstant();

    // Ajout d'un instant
	XERCES_CPP_NAMESPACE::DOMNode *	AddInstant(double dbInstant, int nNbVeh);

    // Sauvegarde des données atmosphériques d'un instant
	void SaveLastInstant();
    void ReleaseLastInstant();
	void Addval(double dbVCO2, double dbVNOx, double dbVPM, double dbCumCO2, double dbCumNOx, double dbCumPM );

	// Sauvegarde des données atmosphériques d'un véhicule (cumul)
	void AddVehicule(int nID, const std::string & strType, const std::string & strEntree, double dbInstCreation, const std::string & strSortie, double dbInstSortie, double dbCumCO2, double dbCumNOx, double dbCumPM, double dbDstParcourue, bool bAddToCreationNode) ;

    // ajout la description d'un véhicule sur sa création
    virtual void AddVehiculeToCreationNode(int nID, const std::string & strLibelle, const std::string & strType, double dbKx, double dbVx, double dbW, const std::string & strEntree, const std::string & strSortie, const std::string & strRoute, double dbInstCreation, const std::string & sVoie, bool bAgressif) {};

	// ajout la description d'un véhicule lors de la sortie d'un véhicule du réseau
	virtual void AddVehiculeToSortieNode(int nID, const std::string & strSortie, double dbInstSortie, double dbVit) {};

    // Sauvegarde des donénes atmosphériques d'un véhicule sur l'instant courant
	void AddVehiculeInst(int nID, double dbCO2, double dbNOx, double dbPM);




};

#endif