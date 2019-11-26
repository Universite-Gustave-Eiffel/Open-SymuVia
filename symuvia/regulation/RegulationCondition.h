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
#ifndef RegulationConditionH
#define RegulationConditionH

#include "RegulationElement.h"

/**
* classe reprÃ©sentant une condition de rÃ©gulation
*/
class RegulationCondition : public RegulationElement
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    RegulationCondition(void);
    virtual ~RegulationCondition(void);

    // ************************************
    // Accesseurs
    // ************************************
    const std::string & getID() {return m_strID;}
    bool isRemplie() {return m_bRemplie;}
    const boost::python::object & getContext() {return m_ContextDict;}

    // ************************************
    // Traitements publics
    // ************************************
    virtual void initialize(Reseau * pReseau, XERCES_CPP_NAMESPACE::DOMNode * pNodeCondition);

    // Test de la condition
    void test(boost::python::dict capteurs);

protected:

    // ************************************
    // Traitements protÃ©gÃ©s
    // ************************************
    virtual std::string getBuiltinModuleName();

private:

    // ************************************
    // Membres privÃ©s
    // ************************************
    // Identifiant de la condition
    std::string m_strID;

    // rÃ©sultat de la condition (remplie ou non)
    bool m_bRemplie;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RegulationConditionH