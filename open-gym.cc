#include "open-gym.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EnvOran");
NS_OBJECT_ENSURE_REGISTERED(EnvOran);

EnvOran::EnvOran(uint64_t nEnb)
    : m_nEnb(nEnb)
{
    SetOpenGymInterface(OpenGymInterface::Get());
    NS_LOG_FUNCTION(this);
}

EnvOran::~EnvOran()
{
    NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
EnvOran::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    return Create<OpenGymDiscreteSpace>(m_nEnb);
}

Ptr<OpenGymSpace>
EnvOran::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    std::cout << "GetObservationSpace: " << m_nEnb<< std::endl;
    std::vector<uint32_t> obs = {m_nEnb+2};
    std::string dtype = ns3::TypeNameGet<double>();
    ns3::Ptr<ns3::OpenGymSpace> observationSpace =
        ns3::CreateObject<ns3::OpenGymBoxSpace>(-500, 1000, obs, dtype);
    return observationSpace;
}

Ptr<OpenGymDataContainer>
EnvOran::GetObservation()
{
    NS_LOG_FUNCTION(this);

    return m_observation;
}

bool
EnvOran::GetGameOver()
{
    NS_LOG_FUNCTION(this);
    return false;
}

bool
EnvOran::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteContainer> discreteAction = DynamicCast<OpenGymDiscreteContainer>(action);
    uint32_t cellId = discreteAction->GetValue();

    m_action = cellId;

    return true;
}

float
EnvOran::GetReward()
{
    NS_LOG_FUNCTION(this);

    return m_reward;

    // float apploss = 0;

    // std::vector<uint64_t> ids = m_db->GetLteUeE2NodeIds();
    // for (auto i : ids)
    // {
    //     std::string query = "SELECT loss FROM nodeapploss WHERE nodeid = " + std::to_string(i) +
    //                         " ORDER BY entryid DESC LIMIT 1";
    //     auto last = m_db->GetCustomQuery(query);
    //     for (auto [key, value] : last)
    //     {
    //         apploss += std::stof(value);
    //     }
    // }
    // apploss /= ids.size();
    // std::cout << "----->" << apploss << std::endl;

    // return 1 - apploss;
}

std::string
EnvOran::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    return "m_name";
}

void
EnvOran::SetObservation(Ptr<OpenGymDataContainer> obs)
{
    NS_LOG_FUNCTION(this);
    m_observation = obs;
}

} // namespace ns3
