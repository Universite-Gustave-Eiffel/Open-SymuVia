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
#ifndef RegulationBriqueH
#define RegulationBriqueH

//rmq. positionnement de cet include avant le reste sinon erreur de compilation sou MAC OS X
#pragma warning( disable : 4244 )
#include <boost/python/dict.hpp>
#pragma warning( default : 4244 )

#include <xercesc/util/XercesVersion.hpp>


namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
}

namespace boost {
    namespace serialization {
        class access;
    }
}

#include <vector>
#include <string>

class Reseau;
class RegulationCapteur;
class RegulationCondition;
class RegulationAction;
class RegulationRestitution;
class TraceDocTrafic;

/**
* Classe reprÃ©sentant une brique de rÃ©gulation
*/
class RegulationBrique
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    RegulationBrique(void);
    RegulationBrique(Reseau * pReseau, const std::string & strID, double dbRespect);
    virtual ~RegulationBrique(void);

    // ************************************
    // Traitements publics
    // ************************************

    // Instancie un nouveau dÃ©tecteur pour la brique
    void addDetecteur(XERCES_CPP_NAMESPACE::DOMNode * pNodeDetecteur);

    // Instancie un nouveau dÃ©clencheur pour la brique
    void addDeclencheur(XERCES_CPP_NAMESPACE::DOMNode * pNodeDeclencheur);

    // Instancie une novuelle action pour la brique
    void addAction(XERCES_CPP_NAMESPACE::DOMNode * pNodeAction);

    // Instancie une novuelle action pour la brique
    void addRestitution(XERCES_CPP_NAMESPACE::DOMNode * pNodeRestitution);

    // exÃ©cute les traitements associÃ©s Ã  la brique pour le pas de temps Ã©coulÃ©
    void run();

    // restitution des sorties Ã©ventuelles dans le fichier de trafic
    void restitution(TraceDocTrafic * pDocTrafic);

    // ************************************
    // Accesseurs
    // ************************************
    std::vector<RegulationCapteur*> & GetLstCapteurs() {return m_LstCapteurs;}
    std::vector<RegulationCondition*> & GetLstConditions() {return m_LstConditions;}
    std::vector<RegulationAction*> & GetLstActions() {return m_LstActions;}
    std::vector<RegulationRestitution*> & GetLstRestitutions() {return m_LstRestitutions;}
    double getTauxRespect() {return m_dbRespect;}
    Reseau * GetNetwork() { return m_pReseau; }

private:

    // ************************************
    // Membres privÃ©s
    // ************************************
    // Pointeur vers le rÃ©seau
    Reseau * m_pReseau;

    // Identifiant
    std::string m_strID;

    // Liste des capteurs utilisÃ©s par la brique de regulation
    std::vector<RegulationCapteur*> m_LstCapteurs;

    // Liste des conditions utilisÃ©es par la brique de regulation
    std::vector<RegulationCondition*> m_LstConditions;

    // Liste des actions utilisÃ©es par la brique de regulation
    std::vector<RegulationAction*> m_LstActions;

    // Liste des actions de restitution utilisÃ©es par la brique de regulation
    std::vector<RegulationRestitution*> m_LstRestitutions;

    // pourcentage de respect de la consigne
    double  m_dbRespect;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RegulationBriqueH