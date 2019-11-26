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
#include "MemoryManagerSymuvia.hpp"

#include <xercesc/util/OutOfMemoryException.hpp>

XERCES_CPP_NAMESPACE_USE

MemoryManager* MemoryManagerSymuvia::getExceptionMemoryManager()
{
  return this;
}

void* MemoryManagerSymuvia::allocate(XMLSize_t size)
{
    void* memptr;
    try {
        memptr = ::operator new(size);
    }
    catch(...) {
        throw OutOfMemoryException();
    }
    if (memptr != NULL) {

		std::map<void*,void*>::iterator ptr = m_list.find(memptr);
		if (ptr == m_list.end())
		{
			m_list[memptr] = memptr;
		}
		else
		{
			m_listerr[memptr] = memptr;
			int zz = 0;
			zz++;
		}
		return memptr;
    }
    throw OutOfMemoryException();
}

void MemoryManagerSymuvia::deallocate(void* p)
{
    if (p)
	{
		//std::deque<void*>::iterator ptr;
		//int i = 0;
		//int idx = -1;
		//for (ptr = m_list.begin(); (ptr != m_list.end()) && (idx < 0); ptr++)
		//{
		//	if (p == (*ptr))
		//	{
		//		idx = i;
		//	}
		//	i ++;
		//}
		std::map<void*,void*>::iterator ptr = m_list.find(p);
		//if (idx < 0)
		if (ptr == m_list.end())
		{
			return;
		}
		else
		{
			//m_list.erase(m_list.begin() + idx);
			m_list.erase(ptr);
		}
		if ((m_list.size() == 5084) && (m_listerr.size() == 49))
		{
			int zz = 0;
			zz++;
		}
        ::operator delete(p);
	}
}

