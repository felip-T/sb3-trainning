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

#ifndef ORAN_LM_CUSTOM_HANDOVER_H
#define ORAN_LM_CUSTOM_HANDOVER_H

#include "open-gym.h"

#include <ns3/ai-module.h>
#include <ns3/oran-data-repository.h>
#include <ns3/oran-lm.h>
#include <ns3/oran-module.h>
#include <ns3/vector.h>
#include <ns3/net-device-container.h>

namespace ns3
{

/**
 * \ingroup oran
 *
 * Logic Module for the Near-RT RIC that issues Commands to handover from
 * an LTE cell to another based on the distance from the UE to the eNBs.
 */
class OranLmCustomHandover : public OranLm
{
  protected:
    /**
     * UE related information.
     */
    struct UeInfo
    {
        uint64_t nodeId; //!< The node ID.
        uint16_t cellId; //!< The cell ID.
        uint16_t rnti;   //!< The RNTI ID.
        double apploss;
        double sinr;
        Vector position; //!< The physical position.
    };

    /**
     * eNB related information.
     */
    struct EnbInfo
    {
        uint64_t nodeId; //!< The node ID.
        uint16_t cellId; //!< The cell ID.
        Vector position; //!< The physical position.
    };

  public:
    static TypeId GetTypeId();
    OranLmCustomHandover();
    ~OranLmCustomHandover() override;

    std::vector<Ptr<OranCommand>> Run(void) override;
    void EndSimulation();
    void SetEnbDevs(std::vector<Ptr<LteEnbNetDevice>> enbDevs);

  private:
    std::vector<OranLmCustomHandover::UeInfo> GetUeInfos(
        Ptr<OranDataRepository> data) const;
    std::vector<OranLmCustomHandover::EnbInfo> GetEnbInfos(
        Ptr<OranDataRepository> data) const;
    std::vector<Ptr<OranCommand>> GetHandoverCommands(
        Ptr<OranDataRepository> data,
        std::vector<OranLmCustomHandover::UeInfo> ueInfos,
        std::vector<OranLmCustomHandover::EnbInfo> enbInfos) const;

    std::vector<Ptr<LteEnbNetDevice>> m_enbDevs;
    uint64_t m_nEnb;

    Ptr<EnvOran> m_model;
}; // class OranLmLte2lteDistanceHandover

} // namespace ns3

#endif
