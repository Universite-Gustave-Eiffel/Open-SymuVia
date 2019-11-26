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
#include "ReaderWriterLock.h"

#include <boost/thread.hpp>

ReaderWriterLock::ReaderWriterLock()
{
    m_bHasWriter = false;
    m_nbReaders = 0;

    // On n'autorise pas plus d'un reader par thread (ne sert à rien de faire plus
    // et permet de se protéger d'une explosion de mémoire consommée si on lance trop de requête le lecture en parallele)
    m_nbMaxReaders = boost::thread::hardware_concurrency();
}

ReaderWriterLock::~ReaderWriterLock()
{
}

void ReaderWriterLock::BeginRead()
{
    boost::unique_lock<boost::mutex> lock(m_Mutex);

    while (m_bHasWriter || m_nbReaders == m_nbMaxReaders)
    {
        m_Cond.wait(lock);
    }

    m_nbReaders++;
}

void ReaderWriterLock::EndRead()
{
    boost::unique_lock<boost::mutex> lock(m_Mutex);

    m_nbReaders--;

    m_Cond.notify_one();
}

void ReaderWriterLock::BeginWrite()
{
    boost::unique_lock<boost::mutex> lock(m_Mutex);

    while (m_bHasWriter || m_nbReaders > 0)
    {
        m_Cond.wait(lock);
    }

    m_bHasWriter = true;
}

void ReaderWriterLock::EndWrite()
{
    boost::unique_lock<boost::mutex> lock(m_Mutex);

    m_bHasWriter = false;

    m_Cond.notify_all();
}
