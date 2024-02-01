#include "enbReporterAvgSinr.h"
#include "oran-report-sinr-enb.h"
#include "oran-report-sinr-ue.h"
#include "train.h"

#include "ns3/applications-module.h"
#include <ns3/buildings-module.h>
#include <ns3/core-module.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/internet-module.h>
#include <ns3/log.h>
#include <ns3/lte-module.h>
#include <ns3/mobility-module.h>
#include <ns3/network-module.h>
#include <ns3/oran-module.h>
#include <ns3/point-to-point-module.h>

#include <math.h>

using namespace ns3;

static std::string s_trafficTraceFile = "traffic-trace.tr";
static std::string s_positionTraceFile = "position-trace.tr";
static std::string s_handoverTraceFile = "handover-trace.tr";

NS_LOG_COMPONENT_DEFINE("OranModel");

void
reportAppLoss(Ptr<OranDataRepository> db)
{
    static double total = 0;
    std::cout << "-----___>HERE" << std::endl;
    if (db->IsActive())
    {
        double avg = 0;
        auto ids = db->GetLteUeE2NodeIds();
        for (auto id : ids)
        {
            double loss = db->GetAppLoss(id);
            std::cout << loss << std::endl;
            avg += loss;
        }
        if (ids.size() > 0)
        {
            avg /= ids.size();
        }
        std::cout << "reward: " << 1 - avg << std::endl;
        total += 1 - avg;
        std::cout << "total: " << total << std::endl;
    }
}

void
NotifyEnbReport(std::vector<Ptr<OranE2NodeTerminator>> reps,
                Ptr<OranModSqlite> db,
                uint16_t cellId,
                uint16_t rnti,
                double rsrp,
                double sinr,
                uint8_t componentCarrierId)
{
    // std::cout << Simulator::Now() << "here" << sinr << std::endl;
    if (std::fmod(Simulator::Now().GetSeconds(), 1.0) > 0.99)
    {
        auto it = std::find_if(reps.begin(), reps.end(), [cellId, db](Ptr<OranE2NodeTerminator> r) {
            return std::get<1>(db->GetLteEnbCellInfo(r->GetE2NodeId())) == cellId;
        });
        if (it != reps.end())
        {
            std::vector<Ptr<OranReporter>> reporters = (*it)->GetReporters();
            Ptr<EnbReporterAvgSinr> rep = DynamicCast<EnbReporterAvgSinr>(
                *std::find_if(reporters.begin(), reporters.end(), [](Ptr<OranReporter> r) {
                    return r->GetInstanceTypeId().GetName() == "ns3::EnbReporterAvgSinr";
                }));

            if (rep)
            {
                rep->UpdateData(sinr, rsrp, rnti);
            }
        }
    }
}

void
PrintGnuplottableBuildingListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename, std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    uint32_t index = 0;
    for (auto it = BuildingList::Begin(); it != BuildingList::End(); ++it)
    {
        ++index;
        Box box = (*it)->GetBoundaries();
        outFile << "set object " << index << " rect from " << box.xMin << "," << box.yMin << " to "
                << box.xMax << "," << box.yMax << " front fs empty " << std::endl;
    }
}

void
PrintGnuplottableUeListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename, std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteUeNetDevice> uedev = node->GetDevice(j)->GetObject<LteUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << uedev->GetImsi() << "\" at " << pos.x << "," << pos.y
                        << " left font \"Helvetica,4\" textcolor rgb \"grey\" front point pt 1 ps "
                           "0.3 lc rgb \"grey\" offset 0,0"
                        << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename, std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteEnbNetDevice> enbdev = node->GetDevice(j)->GetObject<LteEnbNetDevice>();
            if (enbdev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << "set label \"" << enbdev->GetCellId() << "\" at " << pos.x << ","
                        << pos.y
                        << " left font \"Helvetica,4\" textcolor rgb \"white\" front  point pt 2 "
                           "ps 0.3 lc rgb \"white\" offset 0,0"
                        << std::endl;
            }
        }
    }
}

void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid
              << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti
              << std::endl;
}

void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";

    if (!args.empty())
    {
        std::cout << " (" << args << ")";
    }
    std::cout << std::endl;
}

void
RxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    // rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " RX " << p->GetSize()
    //           << std::endl;
}

void
TxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " TX " << p->GetSize()
              << std::endl;
}

void
TracePositions(NodeContainer nodes)
{
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::app);

    posOutFile << Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        Vector pos = nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        posOutFile << " " << pos.x << " " << pos.y;
    }
    posOutFile << std::endl;

    Simulator::Schedule(Seconds(1), &TracePositions, nodes);
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::app);
    std::cout << "----->HANDOVER " << Simulator::Now().GetSeconds() << " " << imsi << " " << cellid
              << " " << rnti << std::endl;
}

int
main(int argc, char* argv[])
{
    bool verbose = false;
    bool useDistance = false;
    bool useTrain = true;
    uint32_t startConfig = 1;
    double lmQueryInterval = 1;
    double txDelay = 0;
    std::string handoverAlgorithm = "ns3::NoOpHandoverAlgorithm";
    Time simTime = Seconds(20);
    std::string dbFileName = "oran-repository.db";
    bool plt = false;

    uint16_t interSiteDistance = 500;
    uint16_t enbDlBandwidth = 25;
    uint16_t enbUlBandwidth = 25;

    bool useHex = false;
    bool useBuildings = false;
    unsigned numberOfBuildings = 30;
    unsigned numberOfBuildingsX = 5;
    uint32_t nEnbs = 9;

    uint64_t seed = 1;
    bool randomSeed = true;

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-distance-lm",
                 "Indicates whether the distance LM should be used or not",
                 useDistance);
    cmd.AddValue("start-config", "The starting configuration", startConfig);
    cmd.AddValue("sim-time", "The duration for which traffic should flow", simTime);
    cmd.AddValue("lm-query-interval", "The LM query interval", lmQueryInterval);
    cmd.AddValue("tx-delay", "The E2 termiantor's transmission delay", txDelay);
    cmd.AddValue("handover-algorithm",
                 "Specify which handover algorithm to use",
                 handoverAlgorithm);
    cmd.AddValue("db-file", "Specify the DB file to create", dbFileName);
    cmd.AddValue("traffic-trace-file",
                 "Specify the traffic trace file to create",
                 s_trafficTraceFile);
    cmd.AddValue("position-trace-file",
                 "Specify the position trace file to create",
                 s_positionTraceFile);
    cmd.AddValue("handover-trace-file",
                 "Specify the handover trace file to create",
                 s_handoverTraceFile);
    cmd.AddValue("use-train", "Specify the handover trace file to create", useTrain);
    cmd.AddValue("plot", "Plot topografy", plt);
    cmd.AddValue("buildings", "Enables buildings", useBuildings);
    cmd.AddValue("hex", "Enables hexagonal Enb allocation", useHex);
    cmd.AddValue("seed", "Seed for random number generator", seed);
    cmd.AddValue("random-seed", "Use random seed", randomSeed);
    cmd.Parse(argc, argv);

    if (randomSeed)
    {
        auto time = std::chrono::high_resolution_clock::now();
        seed = time.time_since_epoch().count();
    }
    std::cout << "Starting configuration: " << seed << std::endl;
    ns3::RngSeedManager::SetSeed(seed);

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Buildings
    double bounds = 100;
    if (useBuildings)
    {
        Ptr<GridBuildingAllocator> buildings = CreateObject<GridBuildingAllocator>();
        buildings->SetAttribute("GridWidth", UintegerValue(numberOfBuildingsX));
        buildings->SetAttribute("LengthX", DoubleValue(100));
        buildings->SetAttribute("LengthY", DoubleValue(100));
        buildings->SetAttribute("DeltaX", DoubleValue(10));
        buildings->SetAttribute("DeltaY", DoubleValue(10));
        buildings->SetAttribute("Height", DoubleValue(6));
        buildings->SetBuildingAttribute("NRoomsX", UintegerValue(2));
        buildings->SetBuildingAttribute("NRoomsY", UintegerValue(4));
        buildings->SetBuildingAttribute("NFloors", UintegerValue(2));
        buildings->SetAttribute("MinX", DoubleValue(0));
        buildings->SetAttribute("MinY", DoubleValue(0));
        BuildingContainer bContainer = buildings->Create(numberOfBuildings);

        lteHelper->SetAttribute("PathlossModel",
                                StringValue("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute("ShadowSigmaExtWalls", DoubleValue(0));
        lteHelper->SetPathlossModelAttribute("ShadowSigmaOutdoor", DoubleValue(1));
        lteHelper->SetPathlossModelAttribute("ShadowSigmaIndoor", DoubleValue(1.5));
        lteHelper->SetPathlossModelAttribute("Los2NlosThr", DoubleValue(1e6));
        lteHelper->SetSpectrumChannelType("ns3::MultiModelSpectrumChannel");
        bounds = 100 * (numberOfBuildings / numberOfBuildingsX) +
                 10 * (numberOfBuildings / (numberOfBuildingsX - 2));
    }
    else
    {
        lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));
    }

    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(enbDlBandwidth));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(enbUlBandwidth));
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    lteHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    // Create a single remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // IP configuration
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Create eNB and UE
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    Ptr<RandomVariableStream> ues = CreateObject<UniformRandomVariable>();
    ues->SetAttribute("Min", DoubleValue(4));
    ues->SetAttribute("Max", DoubleValue(8));
    ueNodes.Create(ues->GetInteger());

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    NetDeviceContainer enbLteDevs;

    if (useHex)
    {
        nEnbs = 9;
        enbNodes.Create(nEnbs);
        mobilityEnb.Install(enbNodes);
        lteHelper->SetEnbAntennaModelType("ns3::ParabolicAntennaModel");
        Ptr<LteHexGridEnbTopologyHelper> enbTopologyHelper =
            CreateObject<LteHexGridEnbTopologyHelper>();
        enbTopologyHelper->SetLteHelper(lteHelper);
        enbTopologyHelper->SetAttribute("InterSiteDistance", DoubleValue(interSiteDistance));
        enbTopologyHelper->SetAttribute("MinX", DoubleValue(interSiteDistance / 2));
        enbTopologyHelper->SetAttribute("GridWidth", UintegerValue(1));
        enbTopologyHelper->SetAttribute("SiteHeight", DoubleValue(25));
        enbLteDevs = enbTopologyHelper->SetPositionAndInstallEnbDevice(enbNodes);
    }
    else
    {
        nEnbs = 3;
        enbNodes.Create(nEnbs);
        lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
        Ptr<ListPositionAllocator> positionAllocEnbs = CreateObject<ListPositionAllocator>();
        positionAllocEnbs->Add(Vector(bounds / 2, bounds / 8, 25));
        positionAllocEnbs->Add(Vector(bounds / 8, bounds - bounds / 8, 25));
        positionAllocEnbs->Add(Vector(bounds - bounds / 8, bounds - bounds / 8, 25));
        // positionAllocEnbs->Add(Vector(bounds / 2, bounds / 2, 25));
        mobilityEnb.SetPositionAllocator(positionAllocEnbs);
        mobilityEnb.Install(enbNodes);
        enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    }

    MobilityHelper mobilityUe;
    mobilityUe.SetMobilityModel("ns3::RandomWalk2dOutdoorMobilityModel",
                                "Bounds",
                                RectangleValue(Rectangle(0, bounds, 0, bounds)),
                                "Speed",
                                StringValue("ns3::UniformRandomVariable[Min=4.0|Max=10.0]"));
    Ptr<RandomBoxPositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator>();
    positionAlloc->SetAttribute(
        "X",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(bounds) + "]"));
    positionAlloc->SetAttribute(
        "Y",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(bounds) + "]"));
    positionAlloc->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));
    mobilityUe.SetPositionAllocator(positionAlloc);
    mobilityUe.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        std::cout << "UE " << i
                  << " position: " << ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition()
                  << std::endl;
    }

    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    switch (startConfig)
    {
    case 0:
        lteHelper->AttachToClosestEnb(ueLteDevs, enbLteDevs);
        break;
    case 1:
        lteHelper->Attach(ueLteDevs, enbLteDevs.Get(0));
        break;
    default:
        NS_ABORT_MSG("Starting configuration " << startConfig << " not supported.");
    }

    lteHelper->AddX2Interface(enbNodes);

    // Install and start applications on UEs and remote host
    uint16_t basePort = 1000;
    ApplicationContainer remoteApps;
    ApplicationContainer ueApps;

    Ptr<RandomVariableStream> onTimeRv = CreateObject<UniformRandomVariable>();
    onTimeRv->SetAttribute("Min", DoubleValue(10));
    onTimeRv->SetAttribute("Max", DoubleValue(10));
    Ptr<RandomVariableStream> offTimeRv = CreateObject<UniformRandomVariable>();
    offTimeRv->SetAttribute("Min", DoubleValue(0));
    offTimeRv->SetAttribute("Max", DoubleValue(0));

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint16_t port = basePort * (i + 1);

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), port));
        ueApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(i)));
        // Enable the tracing of RX packets
        ueApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));

        Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
        remoteApps.Add(streamingServer);
        // Attributes
        streamingServer->SetAttribute(
            "Remote",
            AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
        streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
        streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(streamingServer);
        streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
    }

    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime + Seconds(1));

    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime + Seconds(1));

    // ORAN BEGIN
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    TypeId defaultLmTid = TypeId::LookupByName("ns3::OranLmNoop");

    Ptr<OranLm> defaultLm = nullptr;
    Ptr<OranDataRepository> dataRepository = CreateObject<OranModSqlite>();
    Ptr<OranCmm> cmm = CreateObject<OranCmmHandover>();
    Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
    Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator =
        CreateObject<OranNearRtRicE2Terminator>();

    std::vector<Ptr<LteEnbNetDevice>> enbDevs;
    if (useDistance)
    {
        defaultLmTid = TypeId::LookupByName("ns3::OranLmLte2LteDistanceHandover");
    }
    else if (useTrain)
    {
        for (size_t i = 0; i < enbLteDevs.GetN(); i++)
        {
            enbDevs.push_back(DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i)));
        }
        defaultLmTid = TypeId::LookupByName("ns3::OranLmCustomHandover");
    }

    ObjectFactory defaultLmFactory;
    defaultLmFactory.SetTypeId(defaultLmTid);
    if (useTrain)
    {
        std::cout << "nEnbs: " << nEnbs << std::endl;
        defaultLmFactory.Set("nEnb", UintegerValue(nEnbs));
        defaultLm = defaultLmFactory.Create<OranLm>();
        DynamicCast<OranLmCustomHandover>(defaultLm)->SetEnbDevs(enbDevs);
    }
    else
    {
        defaultLm = defaultLmFactory.Create<OranLm>();
    }

    dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
    defaultLm->SetAttribute("Verbose", BooleanValue(verbose));
    defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

    cmm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

    nearRtRicE2Terminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    nearRtRicE2Terminator->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRicE2Terminator->SetAttribute(
        "TransmissionDelayRv",
        StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));

    nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(defaultLm));
    nearRtRic->SetAttribute("E2Terminator", PointerValue(nearRtRicE2Terminator));
    nearRtRic->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRic->SetAttribute("LmQueryInterval", TimeValue(Seconds(lmQueryInterval)));
    nearRtRic->SetAttribute("ConflictMitigationModule", PointerValue(cmm));

    Simulator::Schedule(Seconds(1), &OranNearRtRic::Start, nearRtRic);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
            CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
        Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator = CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        appLossReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
        remoteApps.Get(idx)->TraceConnectWithoutContext(
            "Tx",
            MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
        ueApps.Get(idx)->TraceConnectWithoutContext(
            "Rx",
            MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

        lteUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteUeTerminator->SetAttribute("RegistrationIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteUeTerminator->SetAttribute("SendIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        lteUeTerminator->AddReporter(locationReporter);
        lteUeTerminator->AddReporter(lteUeCellInfoReporter);
        lteUeTerminator->AddReporter(appLossReporter);
        lteUeTerminator->SetAttribute(
            "TransmissionDelayRv",
            StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));

        lteUeTerminator->Attach(ueNodes.Get(idx));

        Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
    }

    std::vector<Ptr<OranE2NodeTerminator>> enbTerminators;
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranE2NodeTerminatorLteEnb> lteEnbTerminator =
            CreateObject<OranE2NodeTerminatorLteEnb>();
        Ptr<EnbReporterAvgSinr> avgSinrReporter = CreateObject<EnbReporterAvgSinr>();

        avgSinrReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));

        locationReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));

        lteEnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteEnbTerminator->SetAttribute("RegistrationIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteEnbTerminator->SetAttribute("SendIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        lteEnbTerminator->AddReporter(locationReporter);
        lteEnbTerminator->AddReporter(avgSinrReporter);
        lteEnbTerminator->Attach(enbNodes.Get(idx));
        lteEnbTerminator->SetAttribute(
            "TransmissionDelayRv",
            StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));
        Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorLteEnb::Activate, lteEnbTerminator);

        enbTerminators.push_back(lteEnbTerminator);
    }

    Ptr<OranModSqlite> db = DynamicCast<OranModSqlite>(nearRtRic->Data());

    Config::SetDefault("ns3::LteEnbPhy::UeSinrSamplePeriod", UintegerValue(65000));
    Config::SetDefault("ns3::LteUePhy::RsrpSinrSamplePeriod", UintegerValue(65000));

    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/"
                                  "ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                                  MakeBoundCallback(&NotifyEnbReport, enbTerminators, db));

    if (verbose)
    {
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }
    // ORAN END

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedEnb));

    // lteHelper->EnableTraces();

    if (useBuildings)
    {
        BuildingsHelper::Install(ueNodes);
        BuildingsHelper::Install(enbNodes);
    }

    Ptr<RadioEnvironmentMapHelper> remHelper;
    if (plt)
    {
        if (useBuildings)
        {
            PrintGnuplottableBuildingListToFile("buildings.txt");
        }
        PrintGnuplottableEnbListToFile("enbs.txt");
        PrintGnuplottableUeListToFile("ues.txt");

        remHelper = CreateObject<RadioEnvironmentMapHelper>();
        remHelper->SetAttribute("Channel", PointerValue(lteHelper->GetDownlinkSpectrumChannel()));
        remHelper->SetAttribute("OutputFile", StringValue("oran.rem"));
        remHelper->SetAttribute("XMin", DoubleValue(0));
        remHelper->SetAttribute("XMax", DoubleValue(bounds));
        remHelper->SetAttribute("YMin", DoubleValue(0));
        remHelper->SetAttribute("YMax", DoubleValue(bounds));
        remHelper->SetAttribute("Z", DoubleValue(1.5));
        remHelper->Install();
        std::cout << "REMs created" << std::endl;
    }

    std::ofstream trafficOutFile(s_trafficTraceFile, std::ios_base::trunc);
    trafficOutFile.close();
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::trunc);
    posOutFile.close();
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
    hoOutFile.close();

    Simulator::Schedule(Seconds(1), &TracePositions, ueNodes);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    for (Time t = Seconds(2); t < simTime; t += Seconds(1))
    {
        Simulator::Schedule(t, &reportAppLoss, nearRtRic->Data());
    }

    Simulator::Stop(simTime);
    Simulator::Run();
    flowMonitor->SerializeToXmlFile("flowmonitor.xml", false, true);
    Simulator::Destroy();

    if (useTrain)
    {
        DynamicCast<OranLmCustomHandover>(nearRtRic->GetDefaultLogicModule())->EndSimulation();
    }
    return 0;
}
