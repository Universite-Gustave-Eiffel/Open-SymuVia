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
#ifndef RegulationActionH
#define RegulationActionH

#include "RegulationElement.h"

#include <vector>

class RegulationCondition;

/**
* Classe reprÃ©sentant une action de rÃ©gulation
*/
class RegulationAction : public RegulationElement
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    RegulationAction(void);
    virtual ~RegulationAction(void);

    // ************************************
    // Traitements publics
    // ************************************
    virtual void initialize(Reseau * pReseau, XERCES_CPP_NAMESPACE::DOMNode * pNodeAction, const std::vector<RegulationCondition*> & lstConditions);

    // test prÃ©liminaire Ã  l'exÃ©cution de l'action
    bool test();

    // exÃ©cution de l'action Ã©tant donnÃ© les conditions passÃ©es en paramÃ¨tres
    void execute();

protected:

    // ************************************
    // Traitements protÃ©gÃ©s
    // ************************************
    virtual std::string getBuiltinModuleName();


    // ************************************
    // Membres protÃ©gÃ©s
    // ************************************
    // Liste des conditions nÃ©cessaires Ã  l'exÃ©cution de l'action
    std::vector<RegulationCondition*> m_LstConditions;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RegulationActionH