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

#include "Point.h"

#include <math.h>

using namespace SymuCore;

Point::Point()
{
    m_X = 0;
    m_Y = 0;
}

Point::Point(double x, double y)
{
    m_X = x;
    m_Y = y;
}

Point::~Point()
{

}

void Point::setX(double x)
{
    m_X = x;
}

double Point::getX()
{
    return m_X;
}

double Point::getY()
{
    return m_Y;
}

void Point::setY(double y)
{
    m_Y = y;
}

double Point::distanceTo(Point * pOtherPoint) const
{
    double dbResult;
    dbResult = sqrt((m_X - pOtherPoint->m_X)*(m_X - pOtherPoint->m_X) + (m_Y - pOtherPoint->m_Y)*(m_Y - pOtherPoint->m_Y));
    return dbResult;
}
