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
#include "entree.h"

#include "tuyau.h"
#include "SystemUtil.h"
#include "DocTrafic.h"
#include "usage/Position.h"

//---------------------------------------------------------------------------
// Constructeurs, destructeurs et assimilés
//---------------------------------------------------------------------------

// Destructeurs
Entree::~Entree()
{    
}

// Constructeur par défaut
Entree::Entree() : SymuViaTripNode(), ConnectionPonctuel()
{
}

// Constructeurs normal de l'entrée
Entree::Entree(char _Nom[256], Reseau *pR): SymuViaTripNode(_Nom, pR),
    ConnectionPonctuel(_Nom, pR)
{
    // La position de sortie du TripNode vers le réseau est l'entrée elle-même
    m_OutputPosition.SetConnection(this);
}



//================================================================
	void Entree::SortieTrafic
//----------------------------------------------------------------
// Fonction  : Module de sortie des données trafic de l'entrée
// Version du: 16/11/2009
// Historique: 16/11/2009 (C.Bécarie - Tinea)
//             Création
//================================================================
(
	DocTrafic *pXMLDocTrafic
)
{
	int nVeh = 0;

	if( m_LstTuyAv.size()==0 )
		return;

	if( !m_LstTuyAv.front() )
		return;

	for(int nVoie = 0; nVoie < m_LstTuyAv.front()->getNb_voies(); nVoie++)	
	{
		nVeh += (int) m_mapVehEnAttente[m_LstTuyAv.front()].find( nVoie )->second.size();		// Comptage des véhicules en attente
		if( m_mapVehPret[m_LstTuyAv.front()].find( nVoie )->second != -1)						// Ajout du véhicule en attente mais prêt
			nVeh++;
	}

	pXMLDocTrafic->AddSimuEntree(SymuViaTripNode::GetID(), nVeh);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void Entree::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void Entree::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void Entree::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SymuViaTripNode);
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(ConnectionPonctuel);
}