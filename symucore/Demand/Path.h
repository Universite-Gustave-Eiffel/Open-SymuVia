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

#ifndef SYMUCORE_PATH_H
#define SYMUCORE_PATH_H

#include "SymuCoreExports.h"

#include "Graph/Model/Pattern.h"
#include "Graph/Model/Link.h"
#include "Graph/Model/Node.h"

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <vector>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

    class Pattern;
    class Node;
    class Link;
    class Cost;
    class SubPopulation;

class SYMUCORE_DLL_DEF Path {

public:

    Path();
    Path(const std::vector<Pattern*>& listPattern);
    virtual ~Path();

    //getters
    const std::vector<Pattern*>& GetlistPattern() const;
    std::vector<Pattern*>& GetlistPattern();
    void SetListPattern(const std::vector<Pattern *> &listPattern);

    Cost GetPathCost(int iSimulationInstance, double dbStrartingTime, SubPopulation *pSubPopulation, bool shouldStartFromEnd) const;

    bool operator==(const Path & otherPath) const;
    bool operator<(const Path & otherPath) const;

    bool empty() const;

    friend std::ostream& operator<< (std::ostream& stream, const Path& path)
    {
        for(size_t i = 0; i < path.GetlistPattern().size()-1; i++)
        {
            stream << path.GetlistPattern()[i]->toString() << "\\";
            stream << path.GetlistPattern()[i]->getLink()->getDownstreamNode()->getStrName() << "\\";
        }
        stream << path.GetlistPattern()[path.GetlistPattern().size()-1]->toString();

        return stream;
    }

private:

    std::vector<Pattern*>		m_listPattern; //list of pattern used from orign node to destination node
};
}

#pragma warning(pop)

#endif // SYMUCORE_PATH_H
