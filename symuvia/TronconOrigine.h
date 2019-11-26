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
#ifndef TronconOrigineH
#define TronconOrigineH

#include "usage/SymuViaTripNode.h"
#include "Connection.h"

class ZoneDeTerminaison;
class TypeVehicule;

class TronconOrigine : public SymuViaTripNode
{
public:
   TronconOrigine() {m_pZoneParent=NULL; m_pTuyau=NULL;}
    TronconOrigine(Tuyau * pTuyau, ZoneDeTerminaison * pZoneParent);
    virtual ~TronconOrigine();

    // ************************************
    // Traitements publics
    // ************************************
    virtual std::string GetOutputID() const;

    ZoneDeTerminaison * GetZoneParent() {return m_pZoneParent;}

    Tuyau * GetTuyau() {return m_pTuyau;}

    virtual std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > > & GetMapAssignment() {return m_mapAssignment;}

    // ************************************
    // Membres privés
    // ************************************
private:
    ZoneDeTerminaison * m_pZoneParent;

    Tuyau * m_pTuyau;

    // Variable de stockage des données d'affectation
	std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > >		m_mapAssignment;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif //TronconOrigineH
