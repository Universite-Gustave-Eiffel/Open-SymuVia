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
#include "LoadUsageParameters.h"

#include "usage/TripNode.h"

#include "tools.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

XERCES_CPP_NAMESPACE_USE

LoadUsageParameters::LoadUsageParameters()
{
    m_bMonteeDescenteSimultanee = false;
    m_dbTempsMonteeIndividuel = 1;
    m_dbTempsDescenteIndividuel = 1;
    m_MaxLoad = INT_MAX;
    m_CurrentLoad = 0;
}

LoadUsageParameters::~LoadUsageParameters()
{
}

void LoadUsageParameters::Load(DOMNode * pNode, Logger & loadingLogger)
{
    GetXmlAttributeValue(pNode, "temps_montee_individuelle", m_dbTempsMonteeIndividuel, &loadingLogger);
    GetXmlAttributeValue(pNode, "temps_descente_individuelle", m_dbTempsDescenteIndividuel, &loadingLogger);
    GetXmlAttributeValue(pNode, "montee_descente_simultanee", m_bMonteeDescenteSimultanee, &loadingLogger);
    GetXmlAttributeValue(pNode, "charge_max", m_MaxLoad, &loadingLogger);
}

bool    LoadUsageParameters::IsMonteeDescenteSimultanee()
{
    return m_bMonteeDescenteSimultanee;
}

void    LoadUsageParameters::SetMonteeDescenteSimultanee(bool bMonteeDescenteSimultanee)
{
    m_bMonteeDescenteSimultanee = bMonteeDescenteSimultanee;
}

double  LoadUsageParameters::GetTempsMonteeIndividuel()
{
    return m_dbTempsMonteeIndividuel;
}

void    LoadUsageParameters::SetTempsMonteeIndividuel(double dbTempsMontee)
{
    m_dbTempsMonteeIndividuel = dbTempsMontee;
}

double  LoadUsageParameters::GetTempsDescenteIndividuel()
{
    return m_dbTempsDescenteIndividuel;
}

void    LoadUsageParameters::SetTempsDescenteIndividuel(double dbTempsDescente)
{
    m_dbTempsDescenteIndividuel = dbTempsDescente;
}

int  LoadUsageParameters::GetMaxLoad()
{
    return m_MaxLoad;
}

void    LoadUsageParameters::SetMaxLoad(int iMaxLoad)
{
    m_MaxLoad = iMaxLoad;
}

int  LoadUsageParameters::GetCurrentLoad()
{
    return m_CurrentLoad;
}

void    LoadUsageParameters::SetCurrentLoad(int iCurrentLoad)
{
    m_CurrentLoad = iCurrentLoad;
}

double  LoadUsageParameters::ComputeLoadTime(int iLoadQuantity, int iUnloadQuantity)
{
    double dbLoadTime = 0;
    double dbDureeDescente = iUnloadQuantity * GetTempsDescenteIndividuel();
    double dbDureeMontee = iLoadQuantity * GetTempsMonteeIndividuel();
    if(IsMonteeDescenteSimultanee())
    {
        dbLoadTime = std::max<double>(dbDureeDescente, dbDureeMontee);
    }
    else
    {
        dbLoadTime = dbDureeMontee + dbDureeDescente;
    }
    return dbLoadTime;
}

std::vector<Passenger> & LoadUsageParameters::GetPassengers(TripNode * pTripNode)
{
    return m_currentPassengers[pTripNode];
}

template void LoadUsageParameters::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void LoadUsageParameters::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void LoadUsageParameters::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_bMonteeDescenteSimultanee);
    ar & BOOST_SERIALIZATION_NVP(m_dbTempsMonteeIndividuel);
    ar & BOOST_SERIALIZATION_NVP(m_dbTempsDescenteIndividuel);
    ar & BOOST_SERIALIZATION_NVP(m_MaxLoad);
    ar & BOOST_SERIALIZATION_NVP(m_CurrentLoad);
    ar & BOOST_SERIALIZATION_NVP(m_currentPassengers);
}
