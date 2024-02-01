#ifndef OPENGYM_H
#define OPENGYM_H

#include <ns3/ai-module.h>
#include <ns3/oran-module.h>


namespace ns3 {
class EnvOran : public OpenGymEnv
{
    public:
        friend class TrainningLm;
        EnvOran(uint64_t);
        ~EnvOran() override;
        // static TypeId EnvOran::GetTypeId();

        Ptr<OpenGymSpace> GetActionSpace() override;
        Ptr<OpenGymSpace> GetObservationSpace() override;
        std::string GetExtraInfo() override;


        Ptr<OpenGymDataContainer> GetObservation() override;
        float GetReward() override;
        bool GetGameOver() override;
        bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

        uint32_t m_action;
        double m_reward;

        void setEnv(Ptr<OranNearRtRic> ric);
        void SetObservation(Ptr<OpenGymDataContainer> observation);

    private:
        uint32_t m_nEnb;
        Ptr<OranModSqlite> m_db;
        Ptr<OpenGymDataContainer> m_observation;
};
}

#endif /* OPENGYM_H */
