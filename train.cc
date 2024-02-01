/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * NIST-developed software is provided by NIST as a public service. You may
 * use, copy and distribute copies of the software in any medium, provided that
 * you keep intact this entire notice. You may improve, modify and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software.
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO
 * WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY OPERATION OF
 * LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST
 * DOES NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE
 * SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
 * CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 *
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This
 * software is not intended to be used in any situation where a failure could
 * cause risk of injury or damage to property. The software developed by NIST
 * employees is not subject to copyright protection within the United States.
 */

#include "train.h"

#include "open-gym.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/lte-module.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <cfloat>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmCustomHandover");

NS_OBJECT_ENSURE_REGISTERED(OranLmCustomHandover);

TypeId
OranLmCustomHandover::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::OranLmCustomHandover")
                            .SetParent<OranLm>()
                            .AddConstructor<OranLmCustomHandover>()
                            .AddAttribute("nEnb",
                                          "Number of eNBs",
                                          UintegerValue(4),
                                          MakeUintegerAccessor(&OranLmCustomHandover::m_nEnb),
                                          MakeUintegerChecker<uint64_t>());

    return tid;
}

OranLmCustomHandover::OranLmCustomHandover(void)
    : OranLm()
{
    NS_LOG_FUNCTION(this);

    m_model = CreateObject<EnvOran>(3);
    m_name = "OranLmCustomHandover";
}

OranLmCustomHandover::~OranLmCustomHandover(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmCustomHandover::Run(void)
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<UeInfo> ueInfos = GetUeInfos(data);
        std::vector<EnbInfo> enbInfos = GetEnbInfos(data);
        commands = GetHandoverCommands(data, ueInfos, enbInfos);
    }

    // Return the commands.
    return commands;
}

std::vector<OranLmCustomHandover::UeInfo>
OranLmCustomHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    Ptr<OranModSqlite> db = DynamicCast<OranModSqlite>(data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : db->GetLteUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;
        // Get the current cell ID and RNTI of the UE and record it.
        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = db->GetLteUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            // Get the latest location of the UE.
            std::map<Time, Vector> nodePositions =
                db->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location informtaion for this UE
                // so record it for a later analysis.
                double apploss;
                double sinr = -1000;

                try
                {
                    apploss = db->GetAppLoss(ueInfo.nodeId);
                    std::string query =
                        "SELECT sinr FROM sinr WHERE ueid = " + std::to_string(ueId) +
                        " ORDER BY time DESC LIMIT 1";
                    sinr = std::stod(db->GetCustomQuery(query)["SINR"]);
                }
                catch (...)
                {
                    apploss = -1;
                }

                if (apploss != -1)
                {
                    ueInfo.apploss = apploss;
                    ueInfo.sinr = sinr;
                    ueInfo.position = nodePositions.rbegin()->second;
                    ueInfos.push_back(ueInfo);
                }
            }
            else
            {
                NS_LOG_INFO("Could not find LTE UE location for E2 Node ID = " << ueInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE UE cell info for E2 Node ID = " << ueInfo.nodeId);
        }
    }
    return ueInfos;
}

std::vector<OranLmCustomHandover::EnbInfo>
OranLmCustomHandover::GetEnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<EnbInfo> enbInfos;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        EnbInfo enbInfo;
        enbInfo.nodeId = enbId;
        // Get the cell ID of this eNB and record it.
        bool found;
        std::tie(found, enbInfo.cellId) = data->GetLteEnbCellInfo(enbInfo.nodeId);
        if (found)
        {
            // Get all known locations of the eNB.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(enbInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location information for this
                // eNB so record it for a later analysis.
                enbInfo.position = nodePositions.rbegin()->second;
                enbInfos.push_back(enbInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find LTE eNB location for E2 Node ID = " << enbInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE eNB cell info for E2 Node ID = " << enbInfo.nodeId);
        }
    }
    return enbInfos;
}

std::vector<Ptr<OranCommand>>
OranLmCustomHandover::GetHandoverCommands(Ptr<OranDataRepository> data,
                                          std::vector<OranLmCustomHandover::UeInfo> ueInfos,
                                          std::vector<OranLmCustomHandover::EnbInfo> enbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<Ptr<OranCommand>> commands;

    // Compare the location of each active eNB with the location of each active
    // UE and see if that UE is currently being served by the closet cell. If
    // there is a closer eNB to the UE then the currently serving cell then
    // issue a handover command.
    double allApploss = 0;
    for (size_t i = 0; i < ueInfos.size(); i++)
    {
        allApploss += sqrt(ueInfos[i].apploss);
    }
    allApploss /= ueInfos.size();

    for (auto ueInfo : ueInfos)
    {
        double min = DBL_MAX;               // The minimum distance recorded.
        uint64_t oldCellNodeId;             // The ID of the cell currently serving the UE.
        uint16_t newCellId = ueInfo.cellId; // The ID of the closest cell.
        std::vector<double> dists;
        for (const auto& enbInfo : enbInfos)
        {
            // Calculate the distance between the UE and eNB.
            double dist = std::sqrt(std::pow(ueInfo.position.x - enbInfo.position.x, 2) +
                                    std::pow(ueInfo.position.y - enbInfo.position.y, 2) +
                                    std::pow(ueInfo.position.z - enbInfo.position.z, 2));

            LogLogicToRepository("Distance from UE with RNTI " + std::to_string(ueInfo.rnti) +
                                 " in CellID " + std::to_string(ueInfo.cellId) +
                                 " to eNB with CellID " + std::to_string(enbInfo.cellId) + " is " +
                                 std::to_string(dist));

            dists.push_back(dist);

            // Check if the distance is shorter than the current minimum
            if (dist < min)
            {
                // Record the new minimum
                min = dist;
                // Record the ID of the cell that produced the new minimum.
                newCellId = enbInfo.cellId;

                LogLogicToRepository("Distance to eNB with CellID " +
                                     std::to_string(enbInfo.cellId) + " is shortest so far");
            }

            // Check if this cell is the currently serving this UE.
            if (ueInfo.cellId == enbInfo.cellId)
            {
                // It is, so indicate record the ID of the cell that is
                // currently serving the UE.
                oldCellNodeId = enbInfo.nodeId;
            }
        }

        newCellId = -1;
        bool hand = false;

        for (auto a : m_enbDevs)
        {
            auto cell = a->GetCellId();
            if (ueInfo.cellId == cell)
            {
                hand = DynamicCast<LteEnbRrc>(a->GetRrc())->HasUeManager(ueInfo.rnti);
            }
        }

        if (hand)
        {
            Ptr<OpenGymBoxContainer<double>> observation =
                CreateObject<OpenGymBoxContainer<double>>();
            // observation->AddValue(ueInfo.cellId);
            observation->AddValue(1-2*sqrt(ueInfo.apploss));
            observation->AddValue(ueInfo.sinr/50);
            for (auto dist : dists)
            {
                observation->AddValue(dist/(sqrt(2)*100));
            }

            m_model->SetObservation(observation);
            m_model->m_reward = 1 - (2*allApploss);
            m_model->Notify();

            newCellId = m_model->m_action + 1;
        }

        if (newCellId != ueInfo.cellId)
        {
            if (hand)
            {
                // std::cout << "HANDOVER nodeID: " << ueInfo.nodeId << std::endl;
                // std::cout << "----->" << newCellId << " " << ueInfo.cellId << " " << ueInfo.rnti
                Ptr<OranCommandLte2LteHandover> handoverCommand =
                    CreateObject<OranCommandLte2LteHandover>();
                handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));
                handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));
                handoverCommand->SetAttribute("TargetCellId", UintegerValue(newCellId));
                data->LogCommandLm(m_name, handoverCommand);
                commands.push_back(handoverCommand);

                LogLogicToRepository("Closest eNB (CellID " + std::to_string(newCellId) + ")" +
                                     " is different than the currently attached eNB" + " (CellID " +
                                     std::to_string(ueInfo.cellId) + ")." +
                                     " Issuing handover command.");
            }
            // else
            // {
            //     std::cout << "FAIL nodeID: " << ueInfo.nodeId << std::endl;
            // }
        }
    }
    return commands;
}

void
OranLmCustomHandover::EndSimulation()
{
    m_model->NotifySimulationEnd();
}

void
OranLmCustomHandover::SetEnbDevs(std::vector<Ptr<LteEnbNetDevice>> enbDevs)
{
    m_enbDevs = enbDevs;
}
} // namespace ns3
