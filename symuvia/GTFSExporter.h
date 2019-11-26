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
#ifndef GTFSExporterH
#define GTFSExporterH

#include <map>
#include <string>

class Reseau;

class GTFSExporter {

public:
    GTFSExporter();
    virtual ~GTFSExporter();

    // écriture des fichiers GTFS correspondant au réseau passé en paramètre
    bool write(Reseau * pNetwork, const std::map<std::string, std::pair<int,int> > & lstTPTypes);

private:
    void writeAgency(const std::string & outDir);
    bool writeStops(const std::string & outDir, Reseau * pNetwork);
    void writeCalendar(const std::string & outDir);
    void writeCalendarDates(const std::string & outDir);
    void writeRoutes(const std::string & outDir, Reseau * pNetwork, const std::map<std::string, std::pair<int,int> > & lstTPTypes);
    void writeTrips(const std::string & outDir, Reseau * pNetwork);
    void writeStopTimes(const std::string & outDir, Reseau * pNetwork);
    void writeFrequencies(const std::string & outDir, Reseau * pNetwork);

};

#endif // GTFSExporterH