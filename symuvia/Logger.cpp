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
#include "Logger.h"

#include "TimeUtil.h"
#include "regulation/PythonUtils.h"

#include <iostream>
#include <fstream>

Logger::Logger(const std::string & logFile, bool bEnableFileLog, LoggerSeverity severityFilter)
{
    m_pNetwork = NULL;
    m_eNewLine = NewLine;
    m_CurrentSeverityLevel = Info;
    m_FilterSeverityLevel = severityFilter;
    m_fileName = logFile;
    m_bEnableFileLog = bEnableFileLog;
    if(bEnableFileLog && !m_fileName.empty())
    {
        m_pLogStream = new std::ofstream(logFile.c_str());

        SDateTime	tNow;
	    tNow = SDateTime::Now();
        (*this) << tNow.GetDay()<<"/"<<tNow.GetMonth()<<"/"<<tNow.GetYear()<<std::endl;  // inscrit l'heure et la date
	    (*this) << tNow.GetHour()<<":"<<tNow.GetMinute()<<":"<<tNow.GetSecond()<<std::endl<<std::endl;
    }
    else
    {
        m_pLogStream = NULL;
    }
}

Logger::~Logger()
{
    if(m_pLogStream && m_pLogStream->is_open())
    {
        m_pLogStream->close();
    }
    if(m_pLogStream)
    {
        delete m_pLogStream;
    }
}

void Logger::SetNetwork(Reseau * pNetwork)
{
    m_pNetwork = pNetwork;
}

const std::string & Logger::getFileName()
{
    return m_fileName;
}