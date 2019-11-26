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

#include "Path.h"
#include "Graph/Model/Cost.h"

using namespace SymuCore;
using namespace std;

Path::Path()
{
}

Path::Path(const std::vector<Pattern *> &listPattern)
{
    m_listPattern = listPattern;
}

Path::~Path()
{

}

bool Path::operator == (const Path & otherPath) const
{
    return m_listPattern == otherPath.m_listPattern;
}

bool Path::operator<(const Path & otherPath) const
{
    if(m_listPattern.size() != otherPath.m_listPattern.size())
        return m_listPattern.size() < otherPath.m_listPattern.size();

    for(size_t i = 0; i < m_listPattern.size(); i++)
    {
        if (m_listPattern[i]->getUniqueID() != otherPath.m_listPattern[i]->getUniqueID())
        {
            return m_listPattern[i]->getUniqueID() < otherPath.m_listPattern[i]->getUniqueID();
        }
    }

    return false; //Path are the same
}

const vector<Pattern*>& Path::GetlistPattern() const
{
    return m_listPattern;
}

std::vector<Pattern *> &Path::GetlistPattern()
{
    return m_listPattern;
}

bool Path::empty() const
{
    return m_listPattern.empty();
}

void Path::SetListPattern(const std::vector<Pattern *> &listPattern)
{
    m_listPattern = listPattern;
}

Cost Path::GetPathCost(int iSimulationInstance, double dbStrartingTime, SubPopulation *pSubPopulation, bool shouldStartFromEnd) const
{
    Cost newCost;

    if(shouldStartFromEnd)
    {
        // OTK - TODO - bizarre ce test sur +inf ? ca peut arriver ? c'est grave si on renvoie pas +inf ?
        for(std::vector<Pattern *>::const_reverse_iterator itPattern=m_listPattern.rbegin(); itPattern != m_listPattern.rend() && newCost.getTravelTime() != std::numeric_limits<double>::infinity(); itPattern++)
        {
            Pattern* pPatternAvt;
            Pattern* pPatternAmt;
            newCost.plus((*itPattern)->getPatternCost(iSimulationInstance, dbStrartingTime - newCost.getTravelTime(), pSubPopulation));
            if(itPattern+1 != m_listPattern.rend())
            {
                pPatternAvt = (*itPattern);
                pPatternAmt = (*(itPattern + 1));
                if(pPatternAvt->getPatternType() == PT_Undefined || pPatternAmt->getPatternType() == PT_Undefined)
                    continue;

                newCost.plus(pPatternAvt->getLink()->getUpstreamNode()->getPenaltyCost(iSimulationInstance, pPatternAmt, pPatternAvt, dbStrartingTime - newCost.getTravelTime(), pSubPopulation));
            }
        }
    }
    else
    {

        for(std::vector<Pattern *>::const_iterator itPattern=m_listPattern.begin(); itPattern != m_listPattern.end() && newCost.getTravelTime() != std::numeric_limits<double>::infinity(); itPattern++)
        {
            Pattern* pPatternAmt;
            Pattern* pPatternAvt;
            newCost.plus((*itPattern)->getPatternCost(iSimulationInstance, dbStrartingTime + newCost.getTravelTime(), pSubPopulation));
            if(itPattern+1 != m_listPattern.end())
            {
                pPatternAmt = (*itPattern);
                pPatternAvt = (*(itPattern + 1));
                if(pPatternAvt->getPatternType() == PT_Undefined || pPatternAmt->getPatternType() == PT_Undefined)
                    continue;

                newCost.plus(pPatternAmt->getLink()->getDownstreamNode()->getPenaltyCost(iSimulationInstance, pPatternAmt, pPatternAvt, dbStrartingTime + newCost.getTravelTime(), pSubPopulation));
            }
        }
    }

    return newCost;
}


