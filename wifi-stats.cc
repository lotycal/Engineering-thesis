/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
 * Copyright (c) 2024 AGH University of Krakow
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Szymon Szott <szott@agh.edu.pl>
 * Based on he-wifi-network.cc by S. Deronne <sebastien.deronne@gmail.com>
 * Last update: 2024-01-30 12:29
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include <chrono> // For high resolution clock
#include "ns3/arp-cache.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-interface.h"
#include "ns3/wifi-phy-state.h"
#include "ns3/object-vector.h"
#include "ns3/pointer.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/timestamp-tag.h"


// Exercise: 1
//
// This is a simple scenario to measure the performance of an IEEE 802.11ax Wi-Fi network.
//
// Under default settings, the simulation assumes a single station in an infrastructure network:
//
//  STA     AP
//    *     *
//    |     |
//   n0     n1
//
// The user can specify the number of transmitting stations and the MCS value (0-11).
// The scenario assumes a perfect channel and all nodes are placed in the same location.
// All stations generate constant traffic so as to saturate the channel.
// The simulation output is the aggregate network throughput.

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ex1");

// --- PHY PSDU statistics ---
static uint64_t gPsduSuccesses = 0;
static uint64_t gPsduFailures = 0;
static uint64_t gPhyHeaderFailures = 0;
static uint64_t gRxWhileDecoding = 0;
static uint64_t gRxAbortedByTx = 0;
static uint64_t gTotalRxEvents = 0;
static uint64_t gTotalPhyEvents = 0;
static double gSimulationTime = 0.0;
static uint32_t gPacketSize = 0;


static std::map<std::pair<uint32_t, uint32_t>, double> gTxTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gRxTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gCcaTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gTxOppDurationPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduTxPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduRetryPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduFailPerDev;
static std::map<std::pair<uint32_t,uint32_t>, uint32_t> gTxPacketsPerDev;
static std::map<std::pair<uint32_t,uint32_t>, uint32_t> gRxPacketsPerDev;
static std::map<std::pair<uint32_t,uint32_t>, uint64_t> gTxBytesPerDev;
static std::map<std::pair<uint32_t,uint32_t>, uint64_t> gRxBytesPerDev;
static std::map<std::pair<uint32_t,uint32_t>, Time> gFirstRxTimePerDev;
static std::map<std::pair<uint32_t,uint32_t>, Time> gLastRxTimePerDev;
static std::map<uint64_t, Time> gTxTimestampPerSeq;  // sequence → TX time
static std::vector<double> gInterArrivalTimes;       // for jitter
static std::vector<double> gDelays;                  // for delay
static Time gLastRxTimeGlobal;                       // for inter-arrival
static std::pair<uint32_t, uint32_t> ParseNodeDevFromContext(const std::string& ctx);
static double gPhyIdleSec = 0.0, gPhyCcaSec = 0.0, gPhyTxSec = 0.0, gPhyRxSec = 0.0, gPhyOtherSec = 0.0;
static std::map<std::pair<uint32_t, uint32_t>, double> gRxThroughputPerDev;  // throughput per receiver (Mbit/s)
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gPsduSuccessPerDev;
static std::map<Mac48Address, uint64_t> gPhyHeaderFailed;
static std::map<Mac48Address, uint64_t> gRxEventWhileDecoding;
static std::map<Mac48Address, uint64_t> gRxEventAbortedByTx;
static std::map<Mac48Address, uint64_t> gPsduSucceeded;
static std::map<Mac48Address, uint64_t> gPsduFailed;
static std::map<Mac48Address, uint64_t> gBlockAckRx;
static std::map<Mac48Address, uint64_t> gBlockAckReqRx;
static uint64_t gRxTagged = 0, gRxUntagged = 0;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gTxAttemptsPerDev;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gRetriesPerDev;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gFailuresPerDev;

template <typename T>
void IncrementCounter(std::map<Mac48Address, T>& counter, Mac48Address address)
{
  if (counter.find(address) == counter.end())
  {
    counter[address] = 1;
  }
  else
  {
    counter[address]++;
  }
}

// --- PSDU callbacks (success & failure) ---

static void
PhyRxEndHandler(std::string context, Ptr<const Packet> packet)
{
    // --- zliczanie globalnych sukcesów PSDU ---
    gPsduSuccesses++;
    gTotalRxEvents++;

    auto ids = ParseNodeDevFromContext(context);
    auto key = std::make_pair(ids.first, ids.second);
    gPsduSuccessPerDev[key]++;

    // --- szczegółowa analiza nagłówka MAC ---
    WifiMacHeader hdr;
    Ptr<Packet> copy = packet->Copy();

    if (copy->PeekHeader(hdr))
    {
        Mac48Address addr;

        // ACK, BlockAck, RTS, CTS → tylko adres odbiorcy (Addr1)
        if (hdr.IsAck() || hdr.IsBlockAck() || hdr.IsCts() || hdr.IsRts())
        {
            addr = hdr.GetAddr1();
        }
        else
        {
            // pozostałe ramki (np. Data, Management) → adres nadawcy (Addr2)
            addr = hdr.GetAddr2();
        }

        // pomijamy puste lub niepoprawne adresy (same zera, broadcast)
        if (addr != Mac48Address("00:00:00:00:00:00") && addr != Mac48Address::GetBroadcast())
        {
            IncrementCounter(gPsduSucceeded, addr);

            // zliczanie BlockAck i BlockAckReq
            if (hdr.IsBlockAck())
            {
                IncrementCounter(gBlockAckRx, addr);
            }
            else if (hdr.IsBlockAckReq())
            {
                IncrementCounter(gBlockAckReqRx, addr);
            }
        }

        // oznaczanie pakietów z/do tagiem TimestampTag
        TimestampTag ts;
        if (packet->PeekPacketTag(ts))
        {
            gRxTagged++;
        }
        else
        {
            gRxUntagged++;
        }
    }
}




static void CountTxPackets(std::string context, Ptr<const Packet> packet)
{
    auto key = ParseNodeDevFromContext(context);
    gTxPacketsPerDev[key]++;
    gTxBytesPerDev[key] += packet->GetSize();
    gTxTimestampPerSeq[packet->GetUid()] = Simulator::Now();
}

static void DetailedPhyRxDropHandler(std::string context,
                                     Ptr<const ns3::Packet> packet,
                                     ns3::WifiPhyRxfailureReason reason)
{
  WifiMacHeader hdr;
  Mac48Address addr;

  if (packet)
  {
    Ptr<Packet> copy = packet->Copy();
    if (copy->PeekHeader(hdr))
    {
      addr = hdr.GetAddr2();

      switch (reason)
      {
        case ns3::WifiPhyRxfailureReason::L_SIG_FAILURE:
        case ns3::WifiPhyRxfailureReason::HT_SIG_FAILURE:
        case ns3::WifiPhyRxfailureReason::SIG_A_FAILURE:
        case ns3::WifiPhyRxfailureReason::SIG_B_FAILURE:
          IncrementCounter(gPhyHeaderFailed, addr);
          break;

        case ns3::WifiPhyRxfailureReason::BUSY_DECODING_PREAMBLE:
        case ns3::WifiPhyRxfailureReason::PREAMBLE_DETECT_FAILURE:
          IncrementCounter(gRxEventWhileDecoding, addr);
          break;

        case ns3::WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX:
          IncrementCounter(gRxEventAbortedByTx, addr);
          break;

        default:
          IncrementCounter(gPsduFailed, addr);
          break;
      }
    }
  }
}

static void CountRxPackets(std::string context, Ptr<const Packet> packet)
{
    auto key = ParseNodeDevFromContext(context);
    gRxPacketsPerDev[key]++;
    gRxBytesPerDev[key] += packet->GetSize();

    Time now = Simulator::Now();
    if (gFirstRxTimePerDev.find(key) == gFirstRxTimePerDev.end())
    {
        gFirstRxTimePerDev[key] = now;
    }
    gLastRxTimePerDev[key] = now;

        // --- Compute one-way delay if TX timestamp known ---
    auto txIt = gTxTimestampPerSeq.find(packet->GetUid());
    if (txIt != gTxTimestampPerSeq.end())
    {
        double delay = (Simulator::Now() - txIt->second).GetSeconds();
        gDelays.push_back(delay);
    }

    // --- Compute inter-arrival time for jitter ---
    if (gLastRxTimeGlobal.IsZero())
    {
        gLastRxTimeGlobal = Simulator::Now();
    }
    else
    {
        double iat = (Simulator::Now() - gLastRxTimeGlobal).GetSeconds();
        gInterArrivalTimes.push_back(iat);
        gLastRxTimeGlobal = Simulator::Now();
    }

}


void MyRxMonitorSnifferCallback(std::string context,
                                Ptr<const Packet> packet,
                                uint16_t channelFreqMhz,
                                WifiTxVector txVector,
                                MpduInfo aMpdu,
                                SignalNoiseDbm signalNoise,
                                uint16_t staId)
{
    auto ids = ParseNodeDevFromContext(context);
    uint32_t nodeId = ids.first;
    uint32_t devId = ids.second;

    auto key = std::make_pair(nodeId, devId);
    Time now = Simulator::Now();

    if (gFirstRxTimePerDev.find(key) == gFirstRxTimePerDev.end())
    {
        gFirstRxTimePerDev[key] = now;
    }
    gLastRxTimePerDev[key] = now;
}


static void
PhyRxDropHandler(std::string /*context*/, Ptr<const Packet> /*packet*/, WifiPhyRxfailureReason reason)
{
  switch (reason)
  {
    // --- Header-level failures ---
    case WifiPhyRxfailureReason::L_SIG_FAILURE:
    case WifiPhyRxfailureReason::HT_SIG_FAILURE:
    case WifiPhyRxfailureReason::SIG_A_FAILURE:
    case WifiPhyRxfailureReason::SIG_B_FAILURE:
      gPhyHeaderFailures++;
      break;

    // --- RX events while decoding preamble ---
    case WifiPhyRxfailureReason::BUSY_DECODING_PREAMBLE:
    case WifiPhyRxfailureReason::PREAMBLE_DETECT_FAILURE:
      gRxWhileDecoding++;
      break;

    // --- Reception aborted by the start of a TX transmission ---
    case WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX:
      gRxAbortedByTx++;
      break;

    // Actual PSDU-level failures
    default:
      gPsduFailures++;
      break;
  }
    gTotalRxEvents++;
}

static void
MacTxSuccessPerDev(std::string context, Ptr<const Packet> p)
{
  auto key = ParseNodeDevFromContext(context);
  gMpduTxPerDev[key]++;
}

static void
MacTxFailPerDev(std::string context, Ptr<const Packet> p)
{
  auto key = ParseNodeDevFromContext(context);
  gMpduFailPerDev[key]++;
}

void MacTxOkHandler(std::string context, Ptr<const Packet> packet)
{
    auto ids = ParseNodeDevFromContext(context);
    gTxAttemptsPerDev[ids]++;
}

void MacRetryHandler(std::string context, Ptr<const Packet> packet)
{
    auto ids = ParseNodeDevFromContext(context);
    gRetriesPerDev[ids]++;
}

void MacTxFailedHandler(std::string context, Ptr<const Packet> packet)
{
    auto ids = ParseNodeDevFromContext(context);
    gFailuresPerDev[ids]++;
}


static void
PerDevicePhyStateTracker(std::string context, Time /*start*/, Time duration, WifiPhyState state)
{
  auto key = ParseNodeDevFromContext(context);
  double d = duration.GetSeconds();

  switch (state)
  {
    case WifiPhyState::TX:
      gTxTimePerDev[key] += d;
      gTxOppDurationPerDev[key] += d;
      break;

    case WifiPhyState::RX:
      gRxTimePerDev[key] += d;
      break;

    case WifiPhyState::CCA_BUSY:
      gCcaTimePerDev[key] += d;
      break;

    default:
      break;
  }
}



static void PhyStateLogger(ns3::Time /*start*/, ns3::Time duration, ns3::WifiPhyState state)
{
  gTotalPhyEvents++;
  const double d = duration.GetSeconds();
  switch (state)
  {
    case ns3::WifiPhyState::IDLE:      gPhyIdleSec += d; break;
    case ns3::WifiPhyState::CCA_BUSY:  gPhyCcaSec  += d; break;
    case ns3::WifiPhyState::TX:        gPhyTxSec   += d; break;
    case ns3::WifiPhyState::RX:        gPhyRxSec   += d; break;
    default:                           gPhyOtherSec+= d; break;
  }
}

static std::pair<uint32_t, uint32_t> ParseNodeDevFromContext(const std::string& ctx)
{
  size_t nodeStart = ctx.find("/NodeList/") + 10;
  size_t nodeEnd   = ctx.find("/DeviceList/", nodeStart);
  uint32_t nodeId  = std::stoi(ctx.substr(nodeStart, nodeEnd - nodeStart));

  size_t devStart  = nodeEnd + 12;
  size_t devEnd    = ctx.find("/", devStart);
  uint32_t devId   = std::stoi(ctx.substr(devStart, devEnd - devStart));

  return {nodeId, devId};
}

void PopulateARPcache ()
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365));

  // Create ARP entries for each interface
  for (auto i = NodeList::Begin (); i != NodeList::End (); ++i)
    {
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      if (!ip) continue;

      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (auto j = interfaces.Begin (); j != interfaces.End (); ++j)
        {
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          if (!ipIface) continue;
          Ptr<NetDevice> device = ipIface->GetDevice ();
          if (!device) continue;

          Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress ());

          for (uint32_t k = 0; k < ipIface->GetNAddresses (); ++k)
            {
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal ();
              if (ipAddr == Ipv4Address::GetLoopback ())
                continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr));
              entry->MarkAlive (addr);
              entry->MarkPermanent ();
            }
        }
    }

  // Assign the cache to all interfaces
  for (auto i = NodeList::Begin (); i != NodeList::End (); ++i)
    {
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      if (!ip) continue;

      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (auto j = interfaces.Begin (); j != interfaces.End (); ++j)
        {
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          if (ipIface)
            ipIface->SetAttribute ("ArpCache", PointerValue (arp));
        }
    }

  std::cout << "ARP cache populated successfully for all interfaces." << std::endl;
}

int main(int argc, char *argv[])
{

  // Initialize default simulation parameters
  uint32_t nWifi = 1;            // Number of transmitting stations
  double simulationTime = 1;     // Default simulation time [s]
  int mcs = 11;                  // Default MCS (0–11)
  int channelWidth = 20;         // Default channel width [MHz]
  int gi = 800;                  // Default guard interval [ns]
  uint32_t maxPackets = 1;       // Default number of packets per station
  double interval = 1.0;         // Default interval between packets [s]
  uint32_t packetSize = 1472;    // Default packet size [bytes]

  // Parse command line arguments
  CommandLine cmd;
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("mcs", "Use a specific MCS (0–11)", mcs);
  cmd.AddValue("nWifi", "Number of stations", nWifi);
  cmd.AddValue("maxPackets", "Number of packets to send per station", maxPackets);
  cmd.AddValue("interval", "Time interval between packets (s)", interval);
  cmd.AddValue("packetSize", "Size of each packet (bytes)", packetSize);
  cmd.Parse(argc, argv);

  gSimulationTime = simulationTime;
  gPacketSize = packetSize;

  // Print simulation settings to screen
  std::cout << std::endl
            << "Simulating an IEEE 802.11ax network with the following settings:" << std::endl;
  std::cout << "- number of transmitting stations: " << nWifi << std::endl;
  std::cout << "- frequency band: 5 GHz" << std::endl;
  std::cout << "- modulation and coding scheme (MCS): " << mcs << std::endl;
  std::cout << "- channel width: " << channelWidth << " MHz" << std::endl;
  std::cout << "- guard interval: " << gi << " ns" << std::endl;

  // Create stations and an AP
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(nWifi);
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  // Create a default wireless channel and PHY
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  // Create and configure Wi-Fi network
  WifiMacHelper mac;
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);
  
  // Set channel width for given PHY
  //std::string channelStr("{0, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}");
  //phy.Set("ChannelSettings", StringValue(channelStr));
  phy.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));

  //std::ostringstream oss;
  //oss << "HeMcs" << mcs;
  //wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
  //                             "ControlMode", StringValue(oss.str())); // Set MCS

  wifi.SetRemoteStationManager("ns3::IdealWifiManager");
  
  //Ssid ssid = Ssid("ns3-80211ax"); // Set SSID

  mac.SetType("ns3::AdhocWifiMac");
  //mac.SetType("ns3::StaWifiMac",
  //            "Ssid", SsidValue(ssid));

  // Create and configure Wi-Fi interfaces
  NetDeviceContainer staDevice;
  staDevice = wifi.Install(phy, mac, wifiStaNodes);

  //mac.SetType("ns3::ApWifiMac",
  //            "Ssid", SsidValue(ssid));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install(phy, mac, wifiApNode);

  // --- MAC layer statistics helper ---
  WifiTxStatsHelper txStats;
  txStats.Enable(staDevice);
  txStats.Enable(apDevice);

  // Set guard interval on all interfaces of all nodes
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue(NanoSeconds(gi)));

  // Configure mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(wifiApNode);
  mobility.Install(wifiStaNodes);

  // Install an Internet stack
  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(wifiStaNodes);

  // Configure IP addressing
  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterface = address.Assign(staDevice);
  apNodeInterface = address.Assign(apDevice);

  PopulateARPcache();

  phy.EnablePcapAll("wifi-stats");

    // Install applications (traffic generators)
    ApplicationContainer sourceApplications, sinkApplications;
    const uint16_t portNumber = 9;

    // --- Sink on AP ---
    auto apIpv4 = wifiApNode.Get(0)->GetObject<Ipv4>();
    Ipv4Address apAddr = apIpv4->GetAddress(1, 0).GetLocal();
    InetSocketAddress sinkSocket(apAddr, portNumber);

    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkSocket);
    sinkApplications.Add(packetSinkHelper.Install(wifiApNode.Get(0)));

    // --- Client only on the first station: 1 packet ---
    UdpClientHelper client(sinkSocket);
    client.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    client.SetAttribute("Interval", TimeValue(Seconds(interval)));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));


    sourceApplications.Add(client.Install(wifiStaNodes.Get(0)));

  // Configure application start/stop times
  // Note:
  // - source starts transmission at 1.0 s
  // - source stops at simulationTime+1
  // - simulationTime reflects the time when data is sent
  sinkApplications.Start(Seconds(0.0));
  sinkApplications.Stop(Seconds(simulationTime + 1));
  sourceApplications.Start(Seconds(1.0));
  sourceApplications.Stop(Seconds(simulationTime + 1));


  // Define simulation stop time
  Simulator::Stop(Seconds(simulationTime + 1));

  // Print information that the simulation will be executed
  std::clog << std::endl
            << "Starting simulation... ";
  // Record start time
  auto start = std::chrono::high_resolution_clock::now();

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                MakeCallback(&MyRxMonitorSnifferCallback));


  // --- PHY PSDU success/failure tracking ---
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",
                  MakeCallback(&PhyRxEndHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                  MakeCallback(&PhyRxDropHandler));

  // --- Per-device PHY state tracking ---
  Config::Connect(
    "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
    MakeCallback(&PerDevicePhyStateTracker)
  );

  Config::Connect(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
  MakeCallback(&MacTxSuccessPerDev));

  Config::Connect(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
  MakeCallback(&MacTxFailPerDev));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                MakeCallback(&CountTxPackets));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                MakeCallback(&CountRxPackets));

  Config::ConnectWithoutContext(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
  MakeCallback(&PhyStateLogger));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                MakeCallback(&DetailedPhyRxDropHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",
                  MakeCallback(&PhyRxEndHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                  MakeCallback(&MacTxOkHandler));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                  MakeCallback(&MacTxFailedHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                  MakeCallback(&MacRetryHandler));



  // Run the simulation!
  Simulator::Run();

  // Record stop time and count duration
  auto finish = std::chrono::high_resolution_clock::now();
  std::clog << ("done!") << std::endl;
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n\n";

  // Calculate throughput
  double throughput = 0;
  for (uint32_t index = 0; index < sinkApplications.GetN(); ++index) // Loop over all traffic sinks
  {
    uint64_t totalBytesThrough = DynamicCast<PacketSink>(sinkApplications.Get(index))->GetTotalRx(); // Get amount of bytes received
    throughput += ((totalBytesThrough * 8) / (simulationTime * 1000000.0));                          // Mbit/s
  }

  // Print results
  std::cout << "Results: " << std::endl;
  std::cout << "- aggregate throughput: " << throughput << " Mbit/s" << std::endl;

  {
  const double total = gPhyIdleSec + gPhyCcaSec + gPhyTxSec + gPhyRxSec + gPhyOtherSec;
  const double util  = (total > 0.0) ? ((gPhyTxSec + gPhyRxSec + gPhyCcaSec) / total) : 0.0;

  std::cout << "\n=== PHY layer (time share) ===\n";
  std::cout << "IDLE:      " << gPhyIdleSec << " s\n";
  std::cout << "CCA_BUSY:  " << gPhyCcaSec  << " s\n";
  std::cout << "TX:        " << gPhyTxSec   << " s\n";
  std::cout << "RX:        " << gPhyRxSec   << " s\n";
  std::cout << "OTHER:     " << gPhyOtherSec<< " s\n";
  std::cout << "Channel Utilization: " << std::fixed << std::setprecision(2)
            << (util * 100.0) << " %\n";
}

std::cout << "\n=== PHY layer (time share per node/device) ===\n";
for (const auto& kv : gTxTimePerDev)
{
    auto key = kv.first;
    double tx  = gTxTimePerDev[key];
    double rx  = gRxTimePerDev[key];
    double cca = gCcaTimePerDev[key];
    double totalSimTime = simulationTime + 1.0;
    double idle = totalSimTime - (tx + rx + cca);
    if (idle < 0) idle = 0.0;

    std::cout << "Node " << key.first << " | Dev " << key.second << "\n";
    std::cout << "IDLE:      " << std::fixed << std::setprecision(6) << idle << " s\n";
    std::cout << "CCA_BUSY:  " << cca << " s\n";
    std::cout << "TX:        " << tx  << " s\n";
    std::cout << "RX:        " << rx  << " s\n";
    std::cout << "OTHER:     0 s\n\n";
}


// --- MAC layer (MPDU) statistics ---
std::cout << std::endl << "=== MAC layer (MPDU) statistics ===" << std::endl;

uint64_t mpduSuccesses = txStats.GetSuccesses();
uint64_t mpduFailures = txStats.GetFailures();
uint64_t mpduRetries = txStats.GetRetransmissions();

std::cout << "MPDU Successes:     " << mpduSuccesses << std::endl;
std::cout << "MPDU Failures:      " << mpduFailures << std::endl;
std::cout << "MPDU Retransmits:   " << mpduRetries << std::endl;

std::cout << "\n--- PHY PSDU statistics ---\n";
std::cout << "PHY header failures: " << gPhyHeaderFailures << "\n";
std::cout << "RX while decoding preamble: " << gRxWhileDecoding << "\n";
std::cout << "RX aborted by TX:           " << gRxAbortedByTx << "\n";
std::cout << "PSDU successes:       " << gPsduSuccesses << "\n";
std::cout << "PSDU failures:        " << gPsduFailures << "\n";
std::cout << "Total RX events:       " << gTotalRxEvents << "\n";
std::cout << "Total PHY events:      " << gTotalPhyEvents << "\n";

std::cout << "\n*** PSDU Success (per receiver) ***\n";
for (const auto& kv : gPsduSuccessPerDev)
{
    std::cout << "Node " << kv.first.first
              << " | Dev " << kv.first.second
              << " | PSDU Success = " << kv.second << std::endl;
}


std::cout << "\n--- Channel Utilization per node/device ---\n";
for (const auto& kv : gTxTimePerDev)
{
    auto key = kv.first;
    double tx = kv.second;
    double rx = gRxTimePerDev[key];
    double cca = gCcaTimePerDev[key];
    double busy = tx + rx + cca;
    double totalSimTime = simulationTime + 1.0;
    double utilPct = (busy / totalSimTime) * 100.0;
    double txPct = (tx / totalSimTime) * 100.0;
    double rxPct = (rx / totalSimTime) * 100.0;
    double ccaPct = (cca / totalSimTime) * 100.0;

    std::cout << "Node " << key.first
              << " | Dev " << key.second
              << " | TX=" << std::fixed << std::setprecision(6) << tx
              << " s (" << std::setprecision(2) << txPct << "%)"
              << ", RX=" << std::fixed << std::setprecision(6) << rx
              << " s (" << std::setprecision(2) << rxPct << "%)"
              << ", CCA=" << std::fixed << std::setprecision(6) << cca
              << " s (" << std::setprecision(2) << ccaPct << "%)"
              << " | Utilization=" << std::setprecision(2)
              << utilPct << " %\n";
}


std::cout << "\n--- Tx Opportunity Duration per node/device ---\n";
for (const auto& kv : gTxOppDurationPerDev)
{
  auto key = kv.first;
  double txOpp = kv.second;
  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | Tx Opportunity Duration = "
            << std::fixed << std::setprecision(6) << txOpp << " s\n";
}

std::cout << "\n--- MAC per node/device MPDU statistics ---\n";
for (const auto &kv : gMpduTxPerDev)
{
  auto key = kv.first;
  uint64_t tx = kv.second;
  uint64_t retry = gMpduRetryPerDev[key];
  uint64_t fail = gMpduFailPerDev[key];

  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | TX=" << tx
            << " | Retries=" << retry
            << " | Failures=" << fail
            << std::endl;
}

std::cout << "\n--- MAC aggregate transmission statistics ---\n";
uint32_t totalTxAttempts = 0;
uint32_t totalRetries = 0;
uint32_t totalFailures = 0;

for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
{
    Ptr<Node> node = NodeList::GetNode(i);
    for (uint32_t j = 0; j < node->GetNDevices(); ++j)
    {
        Ptr<NetDevice> dev = node->GetDevice(j);
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(dev);
        if (!wifiDev) continue;

        auto key = std::make_pair(i, j);
        if (gTxAttemptsPerDev.count(key))
            totalTxAttempts += gTxAttemptsPerDev[key];
        if (gRetriesPerDev.count(key))
            totalRetries += gRetriesPerDev[key];
        if (gFailuresPerDev.count(key))
            totalFailures += gFailuresPerDev[key];
    }
}

std::cout << "Tx Attempts (aggregate): " << totalTxAttempts << std::endl;
std::cout << "Tx Retries (aggregate):  " << totalRetries << std::endl;
std::cout << "Tx Failures (aggregate): " << totalFailures << std::endl;


std::cout << "\n--- MAC derived performance metrics per node/device ---\n";
for (const auto &kv : gMpduTxPerDev)
{
  auto key = kv.first;
  uint64_t tx = kv.second;
  uint64_t fail = gMpduFailPerDev[key];
  uint64_t totalAttempts = tx + fail;

  double failureRate = 0.0;
  double successRate = 0.0;

  if (totalAttempts > 0)
  {
    failureRate = (static_cast<double>(fail) / totalAttempts) * 100.0;
    successRate = (static_cast<double>(tx) / totalAttempts) * 100.0;
  }

  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | Total Attempts=" << totalAttempts
            << " | Success Rate=" << std::fixed << std::setprecision(2) << successRate << "%"
            << " | Failure Rate=" << failureRate << "%"
            << std::endl;
}

std::cout << "\n--- MAC Throughput per node/device ---\n";
for (const auto &kv : gMpduTxPerDev)
{
  auto key = kv.first;
  uint64_t txPackets = kv.second;
  double throughputMbps = 0.0;

  if (gSimulationTime > 0)
  {
    // bits per second -> megabits per second
    throughputMbps = (txPackets * gPacketSize * 8.0) / (gSimulationTime * 1e6);
  }

  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | TX Packets=" << txPackets
            << " | Throughput=" << std::fixed << std::setprecision(6)
            << throughputMbps << " Mbit/s\n";
}

std::cout << "\n--- Network/Application layer statistics ---\n";
for (auto &entry : gTxPacketsPerDev)
{
    uint32_t node = entry.first.first;
    uint32_t dev = entry.first.second;
    std::cout << "Node " << node << " | Dev " << dev
              << " | Packets TX=" << gTxPacketsPerDev[entry.first]
              << " (" << gTxBytesPerDev[entry.first] << " bytes)"
              << " | RX=" << gRxPacketsPerDev[entry.first]
              << " (" << gRxBytesPerDev[entry.first] << " bytes)"
              << std::endl;
}

std::cout << "\n--- Packet loss and timing statistics ---\n";
uint64_t totalSent = 0, totalReceived = 0;

for (const auto &entry : gTxPacketsPerDev)
{
    auto key = entry.first;
    uint64_t sent = gTxPacketsPerDev[key];
    uint64_t received = gRxPacketsPerDev[key];
    uint64_t lost = (sent > received) ? (sent - received) : 0;
    double lossRatio = 0.0;

    if (sent > 0)
    {
        lossRatio = (static_cast<double>(lost) / sent) * 100.0;
    }

    totalSent += sent;
    totalReceived += received;

    std::cout << "Node " << key.first << " | Dev " << key.second
              << " | Sent=" << sent
              << " | Received=" << received
              << " | Lost=" << lost
              << " | Loss Ratio=" << std::fixed << std::setprecision(2) << lossRatio << "%"
              << std::endl;

    if (gFirstRxTimePerDev.find(key) != gFirstRxTimePerDev.end())
    {
        double firstRx = gFirstRxTimePerDev[key].GetSeconds();
        double lastRx = gLastRxTimePerDev[key].GetSeconds();
        double duration = lastRx - firstRx;

        std::cout << "   ↳ First RX=" << firstRx << " s"
                  << ", Last RX=" << lastRx << " s"
                  << ", Data transfer duration=" << duration << " s"
                  << std::endl;
    }
}

uint64_t totalLost = (totalSent > totalReceived) ? (totalSent - totalReceived) : 0;
double totalLossRatio = (totalSent > 0) ? (static_cast<double>(totalLost) / totalSent) * 100.0 : 0.0;

std::cout << "\nAggregate Packet Stats: Sent=" << totalSent
          << ", Received=" << totalReceived
          << ", Lost=" << totalLost
          << ", Loss Ratio=" << totalLossRatio << "%\n";

std::cout << "\n--- Data transfer duration per receiver (MAC) ---" << std::endl;
for (const auto& kv : gFirstRxTimePerDev)
{
    auto key = kv.first;
    double duration = (gLastRxTimePerDev[key] - kv.second).GetSeconds();

    if (duration <= 0.0)
        continue;

    std::cout << "Node " << key.first << " | Dev " << key.second
              << " | Data transfer duration = "
              << std::fixed;
    if (duration < 0.01)
        std::cout << std::setprecision(3) << duration * 1e6 << " µs";
    else
        std::cout << std::setprecision(6) << duration << " s";
    std::cout << std::endl;
}

std::cout << "\n--- Delay and Jitter statistics ---\n";

// Average delay
if (!gDelays.empty())
{
    double sumDelay = std::accumulate(gDelays.begin(), gDelays.end(), 0.0);
    double avgDelay = sumDelay / gDelays.size();

    std::cout << "Average delay: "
              << std::fixed << std::setprecision(3)
              << (avgDelay < 0.01 ? avgDelay * 1e6 : avgDelay)
              << (avgDelay < 0.01 ? " µs" : " s") << std::endl;
}
else
{
    std::cout << "Average delay: N/A (no received packets)\n";
}


// Jitter (standard deviation of inter-arrival times)
if (gInterArrivalTimes.size() > 1)
{
    double meanIAT = std::accumulate(gInterArrivalTimes.begin(), gInterArrivalTimes.end(), 0.0) / gInterArrivalTimes.size();
    double sqSum = 0.0;
    for (double val : gInterArrivalTimes)
        sqSum += std::pow(val - meanIAT, 2);
    double jitter = std::sqrt(sqSum / (gInterArrivalTimes.size() - 1));

    std::cout << "Jitter (IAT std dev): "
              << std::fixed << std::setprecision(3)
              << (jitter < 0.01 ? jitter * 1e6 : jitter)
              << (jitter < 0.01 ? " µs" : " s") << std::endl;
}
else
{
    std::cout << "Jitter (IAT std dev): N/A (too few packets)\n";
}


std::cout << "\n--- Receiver-side throughput statistics ---\n";

double aggregateRxThroughput = 0.0;
for (auto& kv : gRxBytesPerDev)
{
    auto [nodeId, devId] = kv.first;
    double bytes = static_cast<double>(kv.second);
    double throughputMbps = (bytes * 8.0) / (simulationTime * 1e6);  // Mbit/s
    gRxThroughputPerDev[{nodeId, devId}] = throughputMbps;
    aggregateRxThroughput += throughputMbps;

    std::cout << "Node " << nodeId
              << " | Dev " << devId
              << " | RX Throughput = "
              << std::fixed << std::setprecision(6)
              << throughputMbps << " Mbit/s" << std::endl;

}

std::cout << "Aggregate RX Throughput = "
          << std::fixed << std::setprecision(6)
          << aggregateRxThroughput << " Mbit/s" << std::endl;


std::cout << "\n--- RX timing per node/device ---" << std::endl;
for (const auto& kv : gFirstRxTimePerDev)
{
    uint32_t nodeId = kv.first.first;
    uint32_t devId = kv.first.second;
    double first = kv.second.GetSeconds();
    double last = gLastRxTimePerDev.count(kv.first) ? gLastRxTimePerDev.at(kv.first).GetSeconds() : first;
    double duration = std::max(0.0, last - first);

    std::cout << "Node " << nodeId << " | Dev " << devId
              << " | First RX=" << std::fixed << std::setprecision(6) << first << " s"
              << " | Last RX=" << last << " s"
              << " | Duration=" << duration << " s"
              << std::endl;
}

// --- Block ACK metrics ---
std::cout << "\n--- Block ACK metrics ---" << std::endl;
std::cout << "- BA total:  0" << std::endl;
std::cout << "- BAR total: 0" << std::endl;

std::cout << "\n--- Detailed Block ACK events (per MAC) ---\n";

for (const auto& kv : gBlockAckRx)
    std::cout << "Node " << kv.first << " BlockAck RX = " << kv.second << std::endl;

for (const auto& kv : gBlockAckReqRx)
    std::cout << "Node " << kv.first << " BlockAckReq RX = " << kv.second << std::endl;

for (const auto& kv : gPhyHeaderFailed)
    std::cout << "MAC " << kv.first << " PHY Header Failures = " << kv.second << std::endl;

for (const auto& kv : gRxEventWhileDecoding)
    std::cout << "MAC " << kv.first << " RX While Decoding = " << kv.second << std::endl;

for (const auto& kv : gRxEventAbortedByTx)
    std::cout << "MAC " << kv.first << " RX Aborted By TX = " << kv.second << std::endl;

// --- Total RX events per receiver (PSDU-based) ---
uint32_t totalPsduSuccess = 0;
for (const auto& entry : gPsduSuccessPerDev)
{
    totalPsduSuccess += entry.second;
}
std::cout << "Total RX events per receiver (PSDU-based): "
          << totalPsduSuccess << std::endl;

// --- Aggregate Data Transfer Duration (all) ---
double totalDataTransferDuration = 0.0;
for (const auto& kv : gFirstRxTimePerDev)
{
    double duration = (gLastRxTimePerDev[kv.first] - kv.second).GetSeconds();
    if (duration > 0.0)
    {
        totalDataTransferDuration += duration;
    }
}
std::cout << "Aggregate Data Transfer Duration (all): "
          << std::fixed << std::setprecision(6)
          << totalDataTransferDuration << " s" << std::endl;


  // Clean-up
  Simulator::Destroy();

  return 0;
}
