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
#include "ConnectionPonctuel.h"

#include "vehicule.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/archive/xml_woarchive.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/shared_ptr.hpp>

XERCES_CPP_NAMESPACE_USE

//---------------------------------------------------------------------------
// Constructeurs, destructeurs et assimilés
//---------------------------------------------------------------------------

// destructeur
ConnectionPonctuel::~ConnectionPonctuel()
{    
}

// Constructeur par défaut
ConnectionPonctuel::ConnectionPonctuel()
{   
}

// Constructeur normal
ConnectionPonctuel::ConnectionPonctuel(std::string sID, Reseau* pReseau)
:Connexion(sID, pReseau)
{     
}

void ConnectionPonctuel::DelInsVeh(boost::shared_ptr<Vehicule> pV)
{
    std::deque <boost::shared_ptr<Vehicule>>::iterator  beg, end, it;

    beg = m_LstInsVeh.begin();
    end = m_LstInsVeh.end();

    for (it = beg;it!=end;it++)
    {
        if(pV == (*it) )
        {
            m_LstInsVeh.erase(it);
            return;
        }
    }
}

bool ConnectionPonctuel::Init( DOMNode *pXmlNodeCnx, Logger *pofLog)
{    
    return true;
}

template void ConnectionPonctuel::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void ConnectionPonctuel::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void ConnectionPonctuel::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Connexion);

    ar & BOOST_SERIALIZATION_NVP(m_LstInsVeh);
}