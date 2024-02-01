#include "oran-report-sinr-ue.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/uinteger.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportSinrUeSql");

NS_OBJECT_ENSURE_REGISTERED(OranReportSinrUeSql);

TypeId
OranReportSinrUeSql::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReportSinrUeSql")
                            .SetParent<OranReportSql>()
                            .AddConstructor<OranReportSinrUeSql>()
                            .AddAttribute("ueid",
                                          "Node ID",
                                          UintegerValue(),
                                          MakeUintegerAccessor(&OranReportSinrUeSql::m_ueId),
                                          MakeUintegerChecker<uint64_t>())
                            .AddAttribute("cellid",
                                          "Connected Cell ID",
                                          UintegerValue(),
                                          MakeUintegerAccessor(&OranReportSinrUeSql::m_cellId),
                                          MakeUintegerChecker<uint64_t>())
                            .AddAttribute("RNTI",
                                          "Actual RNTI",
                                          UintegerValue(),
                                          MakeUintegerAccessor(&OranReportSinrUeSql::m_rnti),
                                          MakeUintegerChecker<uint64_t>())
                            .AddAttribute("SINR",
                                          "Actual SINR",
                                          DoubleValue(),
                                          MakeDoubleAccessor(&OranReportSinrUeSql::m_sinr),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("RSRP",
                                          "Actual RSRP",
                                          DoubleValue(),
                                          MakeDoubleAccessor(&OranReportSinrUeSql::m_rsrp),
                                          MakeDoubleChecker<double>());

    return tid;
}

OranReportSinrUeSql::OranReportSinrUeSql()
    : OranReportSql()
{
    m_tableInfo.emplace_back("ueid", "INTEGER");
    m_tableInfo.emplace_back("RSRP", "REAL");
    m_tableInfo.emplace_back("SINR", "REAL");
    m_tableInfo.emplace_back("cellid", "INTEGER");
    m_tableInfo.emplace_back("RNTI", "INTEGER");
    NS_LOG_FUNCTION(this);
}

OranReportSinrUeSql::~OranReportSinrUeSql()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportSinrUeSql::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportSinr("
       << "nodeid=" << GetReporterE2NodeId() << ";time=" << time.GetTimeStep() << ";ueid=" << m_ueId
       << ";RNTI=" << m_rnti << ";cellid=" << m_cellId << ";sinr=" << m_sinr << ";rsrp=" << m_rsrp
       << ")";

    return ss.str();
}

double
OranReportSinrUeSql::GetSinr() const
{
    NS_LOG_FUNCTION(this);

    return m_sinr;
}

double
OranReportSinrUeSql::GetRsrp() const
{
    NS_LOG_FUNCTION(this);

    return m_rsrp;
}

std::vector<std::tuple<std::string, std::string>>
OranReportSinrUeSql::GetTableInfo()
{
    NS_LOG_FUNCTION(this);
    return m_tableInfo;
}

std::string
OranReportSinrUeSql::GetTableName()
{
    NS_LOG_FUNCTION(this);
    return "sinr";
}
} // namespace ns3
