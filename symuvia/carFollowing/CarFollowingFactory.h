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
#ifndef CarFollowingFactoryH
#define CarFollowingFactoryH

#include <xercesc/util/XercesVersion.hpp>

#include <boost/shared_ptr.hpp>

#include <vector>

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
}

namespace boost {
    namespace serialization {
        class access;
    }
}

class AbstractCarFollowing;
class Reseau;
class Vehicule;
class ScriptedCarFollowing;

class CarFollowingFactory
{
public:
    CarFollowingFactory();
    CarFollowingFactory(Reseau * pNetwork);
    virtual ~CarFollowingFactory();

    // Ajoute un modèle de loi de poursuite
    void addCarFollowing(XERCES_CPP_NAMESPACE::DOMNode * pNodeCarFollowing);

    AbstractCarFollowing * buildCarFollowing(Vehicule * pVehicle);

    AbstractCarFollowing * copyCarFollowing(Vehicule * pVehicle, AbstractCarFollowing * pOriginalCarFollowing);

    ScriptedCarFollowing* GetScriptedCarFollowingFromID(const std::string & strLawID);

private:
    Reseau * m_pNetwork;

    std::vector<ScriptedCarFollowing*> m_LstScriptedLaws;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);

};

#endif // CarFollowingFactoryH
