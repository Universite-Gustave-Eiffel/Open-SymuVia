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

﻿#pragma once

#ifndef SYMUCORE_TIMEFRAME_H
#define SYMUCORE_TIMEFRAME_H

#include <boost/shared_ptr.hpp>
#include <vector>

#include <math.h>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

template<class T>
class TimeFrame // TimeFrame is a structure that represent a frame of time between two instant for a templated class T
{
public:
	//constructor
    TimeFrame(double tBeginTime, double tEndTime, boost::shared_ptr<T> pData)
	{
		m_dBeginTime = tBeginTime;
		m_dEndTime = tEndTime;
        m_pData = pData;
	}

    virtual ~TimeFrame() {}

    double getStartTime() const
    {
        return m_dBeginTime;
    }

    double getEndTime() const
    {
        return m_dEndTime;
    }

    T* getData() const
    {
        return m_pData.get();
    }

    bool contains(double t) const
    {
        return t >= m_dBeginTime && t <= m_dEndTime;
    }

private:
	double                  m_dBeginTime; //time of the beginning of the frame
	double                  m_dEndTime;//time of the end of the frame
    boost::shared_ptr<T>    m_pData;
};

template <class T>
class ListTimeFrame {

public:

    ListTimeFrame() {}

    virtual ~ListTimeFrame() {}

    void addTimeFrame(double tBeginTime, double tEndTime, boost::shared_ptr<T> pData)
    {
        m_listTimeFrame.push_back(TimeFrame<T>(tBeginTime, tEndTime, pData));
    }

    T* getData(double t) const
    {
        // we assume that each timeFrame last the same time (except the last one if assignment period is not a multiple of the travel times update)
        // rmk : thise won'y work if we keep history of the last assignment periods here !!
        double beginTime = m_listTimeFrame[0].getStartTime();
        double periodTime = m_listTimeFrame[0].getEndTime() - beginTime;
        int index = (int)floor( (t - beginTime) / periodTime );

        index = std::max(0, index);
        index = std::min((int)m_listTimeFrame.size()-1, index);

        return m_listTimeFrame[index].getData();
    }

    T* getData(size_t i) const
    {
        return m_listTimeFrame[i].getData();
    }

    const TimeFrame<T> & getTimeFrame(size_t i) const
    {
        return m_listTimeFrame[i];
    }

    size_t size() const
    {
        return m_listTimeFrame.size();
    }

    void clear()
    {
        m_listTimeFrame.clear();
    }



private:
	std::vector<TimeFrame<T> >			m_listTimeFrame; //list of all the time frames for the templated class

};
}

#pragma warning(pop)

#endif // SYMUCORE_TIMEFRAME_H
