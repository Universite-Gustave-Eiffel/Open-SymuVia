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
#include "TronconOrigine.h"

#include "ZoneDeTerminaison.h"
#include "tuyau.h"
#include "vehicule.h"

#include <boost/serialization/map.hpp>
#include <boost/serialization/deque.hpp>

TronconOrigine::TronconOrigine(Tuyau * pTuyau, ZoneDeTerminaison * pZoneParent):
SymuViaTripNode(pTuyau->GetLabel(), pTuyau->GetReseau())
{
    m_pZoneParent = pZoneParent;
    m_pTuyau = pTuyau;

    m_OutputPosition.SetLink(m_pTuyau);
}

TronconOrigine::~TronconOrigine()
{
}


// Identification de l'origine
std::string TronconOrigine::GetOutputID() const
{
    return m_pZoneParent ? (m_pZoneParent->GetOutputID() + " : " + SymuViaTripNode::GetOutputID()) : SymuViaTripNode::GetOutputID();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void TronconOrigine::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void TronconOrigine::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void TronconOrigine::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SymuViaTripNode);

    ar & BOOST_SERIALIZATION_NVP(m_pZoneParent);
    ar & BOOST_SERIALIZATION_NVP(m_pTuyau);
    ar & BOOST_SERIALIZATION_NVP(m_mapAssignment);
}