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

#ifndef SYMUCORE_PREDEFINED_PATH_H
#define SYMUCORE_PREDEFINED_PATH_H

#include "Path.h"

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

    class Pattern;
    class Node;

class SYMUCORE_DLL_DEF PredefinedPath {

public:

    PredefinedPath();
    PredefinedPath(const std::vector<Pattern*>& listPattern, Node * pJunction, const std::string & name, double dbCoeff);
    virtual ~PredefinedPath();

    double getCoeff() const;
    const Path & getPath() const;
    const std::string & getStrName() const;
    Node * getJunction() const;

private:

    Path m_Path; // predefined path
    Node * m_pJunction; // junction for the predefined path if the origin touches the destination on one node
    std::string m_strName; // name of the predefined path
    double m_dbCoeff;// assignment coefficient of the predefined path
};
}

#pragma warning(pop)

#endif // SYMUCORE_PREDEFINED_PATH_H
