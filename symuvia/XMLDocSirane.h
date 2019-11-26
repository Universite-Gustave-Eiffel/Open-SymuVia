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
#ifndef XMLDocSiraneH
#define XMLDocSiraneH

#include <xercesc/util/XercesVersion.hpp>

namespace XERCES_CPP_NAMESPACE {
    class DOMElement;
    class DOMDocument;
    class DOMNode;
};

namespace boost {
    namespace serialization {
        class access;
    }
}

#include <string>
#include <vector>

struct Point;
class DOMLSSerializerSymu;
class Reseau;

// Types de cellule (segment de tronçon ou brique)
typedef enum { CELL_BRIN = 0, CELL_NOEUD = 1} eTypeCelluleSirane;
typedef enum { UNDEFINED_PHASE = -1, ACCELERATION_PHASE = 1, STABILIZATION_PHASE = 2, DECELERATION_PHASE = 3} ePhaseCelluleSirane;



/*===========================================================================================*/
/* Classe de modélisation du document XML des données nécessaire à Sirane                    */
/*===========================================================================================*/
class XMLDocSirane
{
private:

	std::string				m_strFilename;
	DOMLSSerializerSymu		* m_XmlWriter;

    Reseau *                m_pNetwork;

	XERCES_CPP_NAMESPACE::DOMNode   *m_XmlNodeSimulation;

    // noeud de la période courante en cours d'écriture
    XERCES_CPP_NAMESPACE::DOMElement    *m_XmlNodePeriode;

    // noeud de la cellule courante en cours d'écriture
    XERCES_CPP_NAMESPACE::DOMElement    *m_XmlNodeCell;

	// Le document XML
	XERCES_CPP_NAMESPACE::DOMDocument   *pXMLDoc;

    bool                    m_bReinitialized;       // Indique si le writer courant est issu d'un restore de snapshot

    // Compteur du nombre de cellules
    int                     m_nNbCells;

public:
    // Constructeurs
    XMLDocSirane();
    XMLDocSirane(Reseau * pNetwork, XERCES_CPP_NAMESPACE::DOMDocument * pXMLDocument);
	~XMLDocSirane();

    // Initialisation
	void Init(const std::string & strFilename, const std::vector<Point> & boundingRect);

    // Re-initialisation (après une désérialisation par exemple)
    virtual void ReInit(const std::string & result);

    // Fin
	void Terminate();

    // Mise en veille (prise snapshot pour reprise plus tard)
    virtual void Suspend();

    // Ajout de la description d'une cellule sirane
	virtual void AddCellule(const std::string & strLibelle, eTypeCelluleSirane type, const std::vector<Point*> & lstPts, ePhaseCelluleSirane iPhase);

    // Ajout d'une nouvelle période d'agrégation
    virtual void AddPeriod(double dbInstantDebut, double dbInstantFin);

    // Ajout des résultats d'une cellule sur la période
    virtual void AddCellSimu(const std::string & strLibelle);

    // Ajout des résultats d'une cellule sur la période pour un type de véhicule
    virtual void AddCellSimuForType(const std::string & strType, const std::vector<double> & vitMins, const std::vector<double> & vitMaxs, const std::vector<double> & durees);

    // Fin de la sauvegarde de la période d'agrégation courante
    virtual void ClosePeriod();



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif