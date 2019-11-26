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

#include "AbstractPenalty.h"

using namespace SymuCore;

AbstractPenalty::AbstractPenalty()
{
    m_pNode = NULL;
}

AbstractPenalty::AbstractPenalty(Node* pParentNode, const PatternsSwitch & patternsSwitch)
{
    m_pNode = pParentNode;
    m_PatternsSwitch = patternsSwitch;
}

AbstractPenalty::~AbstractPenalty()
{

}

Node * AbstractPenalty::getNode() const
{
    return m_pNode;
}

const PatternsSwitch & AbstractPenalty::getPatternsSwitch() const
{
    return m_PatternsSwitch;
}
