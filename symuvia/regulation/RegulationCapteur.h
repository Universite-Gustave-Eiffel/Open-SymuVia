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
#ifndef RegulationCapteurH
#define RegulationCapteurH

#include "RegulationElement.h"

/**
* classe reprÃ©sentant un capteur de rÃ©gulation
*/
class RegulationCapteur : public RegulationElement
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    RegulationCapteur(void);
    virtual ~RegulationCapteur(void);

    // ************************************
    // Accesseurs
    // ************************************
    const std::string & getID() {return m_strID;}
    const boost::python::object & getResults() {return m_Result;}

    // ************************************
    // Traitements publics
    // ************************************
    virtual void initialize(Reseau * pReseau, XERCES_CPP_NAMESPACE::DOMNode * pNodeCapteur);

    // mise Ã  jour des mesures du capteur
    void update();

protected:

    // ************************************
    // Traitements protÃ©gÃ©s
    // ************************************
    virtual std::string getBuiltinModuleName();

private:

    // ************************************
    // Membres privÃ©s
    // ************************************
    // Identifiant du capteur
    std::string m_strID;

    // RÃ©sultats de la fonction associÃ©e au capteur pour le PDT courant
    boost::python::object m_Result;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RegulationCapteurH