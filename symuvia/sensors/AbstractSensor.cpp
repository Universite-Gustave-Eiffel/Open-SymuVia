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
#include "AbstractSensor.h"

#include <boost/archive/xml_woarchive.hpp>
#include <boost/archive/xml_wiarchive.hpp>

AbstractSensorSnapshot::~AbstractSensorSnapshot()
{
}

AbstractSensor::AbstractSensor()
{
}

AbstractSensor::AbstractSensor(const std::string & strNom)
{
    m_strNom = strNom;
}

AbstractSensor::~AbstractSensor()
{
}

void AbstractSensor::ResetData()
{
}

std::string AbstractSensor::GetUnifiedID()
{
    return m_strNom;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template void AbstractSensor::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void AbstractSensor::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void AbstractSensor::serialize(Archive& ar, const unsigned int version)
{
	ar & BOOST_SERIALIZATION_NVP(m_strNom);
}

template void AbstractSensorSnapshot::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void AbstractSensorSnapshot::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void AbstractSensorSnapshot::serialize(Archive& ar, const unsigned int version)
{
}