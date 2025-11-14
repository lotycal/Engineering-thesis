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
#include <iomanip>
#include "ns3/wifi-static-setup-helper.h"


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

//  GLOBAL STATISTICS AND DATA STRUCTURES

// Simulation configuration
static double   gSimulationTime = 0.0;
static uint32_t gPacketSize     = 0;

//  PHY LAYER (PSDU-level) STATISTICS
static uint64_t gPsduSuccesses      = 0;
static uint64_t gPsduFailures       = 0;
static uint64_t gPhyHeaderFailures  = 0;
static uint64_t gRxWhileDecoding    = 0;
static uint64_t gRxAbortedByTx      = 0;
static uint64_t gTotalRxEvents      = 0;
static uint64_t gTotalPhyEvents     = 0;

// Per-device PSDU success counters
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gPsduSuccessPerDev;

// Per-address PHY reception breakdown
static std::map<Mac48Address, uint64_t> gPhyHeaderFailed;
static std::map<Mac48Address, uint64_t> gRxEventWhileDecoding;
static std::map<Mac48Address, uint64_t> gRxEventAbortedByTx;
static std::map<Mac48Address, uint64_t> gPsduSucceeded;
static std::map<Mac48Address, uint64_t> gPsduFailed;
static std::map<Mac48Address, uint64_t> gBlockAckRx;
static std::map<Mac48Address, uint64_t> gBlockAckReqRx;
static uint64_t gRxTagged   = 0;
static uint64_t gRxUntagged = 0;

//  PHY STATE TIME STATISTICS (per node/device)
static std::map<std::pair<uint32_t, uint32_t>, double> gTxTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gRxTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gCcaTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gIdleTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, double> gTxOppDurationPerDev;

//  MAC LAYER STATISTICS
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduTxPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduRetryPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gMpduFailPerDev;

// Aggregate MAC transmission metrics
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gTxAttemptsPerDev;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gRetriesPerDev;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> gFailuresPerDev;

//  APPLICATION / NETWORK LAYER STATISTICS
static std::map<std::pair<uint32_t, uint32_t>, uint32_t> gTxPacketsPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint32_t> gRxPacketsPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gTxBytesPerDev;
static std::map<std::pair<uint32_t, uint32_t>, uint64_t> gRxBytesPerDev;

// RX timing (per node/device)
static std::map<std::pair<uint32_t, uint32_t>, Time> gFirstRxTimePerDev;
static std::map<std::pair<uint32_t, uint32_t>, Time> gLastRxTimePerDev;
static Time gLastRxTimeGlobal;

// TX timestamps for delay calculation
static std::map<uint64_t, Time> gTxTimestampPerSeq;

// Delay and jitter
static std::vector<double> gDelays;
static std::vector<double> gInterArrivalTimes;

// Throughput per receiver (Mbit/s)
static std::map<std::pair<uint32_t, uint32_t>, double> gRxThroughputPerDev;

//  FRAME TYPE CLASSIFICATION COUNTERS

// Dropped frame types (PHY RX drop classification)
static uint64_t gDroppedDataFrames      = 0;
static uint64_t gDroppedAckFrames       = 0;
static uint64_t gDroppedRtsFrames       = 0;
static uint64_t gDroppedCtsFrames       = 0;
static uint64_t gDroppedBlockAckFrames  = 0;

// Successfully received frame types (PHY RX end classification)
static uint64_t gTotalDataFrames        = 0;
static uint64_t gTotalAckFrames         = 0;
static uint64_t gTotalRtsFrames         = 0;
static uint64_t gTotalCtsFrames         = 0;
static uint64_t gTotalBlockAckFrames    = 0;

// Successfully transmitted frame types (PHY TX classification)
static uint64_t gTotalTxDataFrames      = 0;
static uint64_t gTotalTxAckFrames       = 0;
static uint64_t gTotalTxRtsFrames       = 0;
static uint64_t gTotalTxCtsFrames       = 0;
static uint64_t gTotalTxBlockAckFrames  = 0;

//  HELPER DECLARATIONS
static std::pair<uint32_t, uint32_t> ParseNodeDevFromContext(const std::string& ctx);

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

// PSDU callbacks (success & failure)

static void
PhyRxEndHandler(std::string context, Ptr<const Packet> packet)
{
     // --- counting global PSDU successes ---
    gPsduSuccesses++;
    gTotalRxEvents++;

    auto ids = ParseNodeDevFromContext(context);
    auto key = std::make_pair(ids.first, ids.second);
    gPsduSuccessPerDev[key]++;

    // --- detailed MAC header analysis ---
    WifiMacHeader hdr;
    Ptr<Packet> copy = packet->Copy();

    if (copy->PeekHeader(hdr))
    {
        Mac48Address addr;

                // --- classify received frame type ---
        if (hdr.IsData())
        {
            gTotalDataFrames++;
        }
        else if (hdr.IsAck())
        {
            gTotalAckFrames++;
        }
        else if (hdr.IsRts())
        {
            gTotalRtsFrames++;
        }
        else if (hdr.IsCts())
        {
            gTotalCtsFrames++;
        }
        else if (hdr.IsBlockAck())
        {
            gTotalBlockAckFrames++;
        }

        // ACK, BlockAck, RTS, CTS → only receiver address (Addr1)
        if (hdr.IsAck() || hdr.IsBlockAck() || hdr.IsCts() || hdr.IsRts())
        {
            addr = hdr.GetAddr1();
        }
        else
        {
            // other frames (e.g., Data, Management) → sender address (Addr2)
            addr = hdr.GetAddr2();
        }

        // skip empty or invalid addresses (all zeros, broadcast)
        if (addr != Mac48Address("00:00:00:00:00:00") && addr != Mac48Address::GetBroadcast())
        {
            IncrementCounter(gPsduSucceeded, addr);

            // counting BlockAck and BlockAckReq
            if (hdr.IsBlockAck())
            {
                IncrementCounter(gBlockAckRx, addr);
            }
            else if (hdr.IsBlockAckReq())
            {
                IncrementCounter(gBlockAckReqRx, addr);
            }
        }

        // marking packets with/without TimestampTag
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


// PHY TX End Handler
static void
PhyTxEndHandler(std::string context, Ptr<const Packet> packet)
{
    WifiMacHeader hdr;
    Ptr<Packet> copy = packet->Copy();
    if (copy->PeekHeader(hdr))
    {
        if (hdr.IsData())
        {
            gTotalTxDataFrames++;
        }
        else if (hdr.IsAck())
        {
            gTotalTxAckFrames++;
        }
        else if (hdr.IsRts())
        {
            gTotalTxRtsFrames++;
        }
        else if (hdr.IsCts())
        {
            gTotalTxCtsFrames++;
        }
        else if (hdr.IsBlockAck())
        {
            gTotalTxBlockAckFrames++;
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

      // --- classify frame type ---
      if (hdr.IsData())
      {
        gDroppedDataFrames++;
      }
      else if (hdr.IsAck())
      {
        gDroppedAckFrames++;
      }
      else if (hdr.IsRts())
      {
        gDroppedRtsFrames++;
      }
      else if (hdr.IsCts())
      {
        gDroppedCtsFrames++;
      }
      else if (hdr.IsBlockAck())
      {
        gDroppedBlockAckFrames++;
      }

      // existing reason-based counting
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

        // Compute one-way delay if TX timestamp known
    auto txIt = gTxTimestampPerSeq.find(packet->GetUid());
    if (txIt != gTxTimestampPerSeq.end())
    {
        double delay = (Simulator::Now() - txIt->second).GetSeconds();
        gDelays.push_back(delay);
    }

    // Compute inter-arrival time for jitter
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


// static void
// PhyRxDropHandler(std::string /*context*/, Ptr<const Packet> /*packet*/, WifiPhyRxfailureReason reason)
// {
//   switch (reason)
//   {
//     // --- Header-level failures ---
//     case WifiPhyRxfailureReason::L_SIG_FAILURE:
//     case WifiPhyRxfailureReason::HT_SIG_FAILURE:
//     case WifiPhyRxfailureReason::SIG_A_FAILURE:
//     case WifiPhyRxfailureReason::SIG_B_FAILURE:
//       gPhyHeaderFailures++;
//       break;

//     // --- RX events while decoding preamble ---
//     case WifiPhyRxfailureReason::BUSY_DECODING_PREAMBLE:
//     case WifiPhyRxfailureReason::PREAMBLE_DETECT_FAILURE:
//       gRxWhileDecoding++;
//       break;

//     // --- Reception aborted by the start of a TX transmission ---
//     case WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX:
//       gRxAbortedByTx++;
//       break;

//     // Actual PSDU-level failures
//     default:
//       gPsduFailures++;
//       break;
//   }
//     gTotalRxEvents++;
// }

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
    case WifiPhyState::IDLE:
      gIdleTimePerDev[key] += d;
      break;

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

  // === Create stations and an AP ===
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(nWifi);
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  // === Create PHY and channel ===
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  // === General Wi-Fi configuration ===
  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ax); // <-- HE (High Efficiency, czyli 802.11ax)
  wifi.SetRemoteStationManager("ns3::IdealWifiManager");

  WifiMacHelper mac;
  Ssid ssid = Ssid("ns3-80211ax");

  // === STA/AP DISABLED FOR AD-HOC MODE ===
  // mac.SetType("ns3::StaWifiMac",
  //               "Ssid", SsidValue(ssid),
  //               "ActiveProbing", BooleanValue(false));
  // NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);

  // mac.SetType("ns3::ApWifiMac",
  //               "Ssid", SsidValue(ssid));
  // NetDeviceContainer apDevices = wifi.Install(phy, mac, wifiApNode);

  // === AD-HOC MODE ===
  mac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);
  NetDeviceContainer apDevices = wifi.Install(phy, mac, wifiApNode);


  // === STATIC SETUP (802.11ax Block Ack) ===
  Ptr<WifiNetDevice> apDev = DynamicCast<WifiNetDevice>(apDevices.Get(0));
  Ptr<WifiNetDevice> staDev = DynamicCast<WifiNetDevice>(staDevices.Get(0));

  NS_ASSERT(apDev != nullptr && staDev != nullptr);

  std::cout << "AP device: " << apDev << ", STA device: " << staDev << std::endl;

  //WifiStaticSetupHelper::SetStaticAssociation(apDev, staDevices);
  //WifiStaticSetupHelper::SetStaticBlockAck(apDev, staDevices, {0});

  // === PHY configuration ===
  phy.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));

  // === TX stats helper ===
  WifiTxStatsHelper txStats;
  txStats.Enable(staDevices);
  txStats.Enable(apDevices);

  // === Guard interval ===
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
              TimeValue(NanoSeconds(gi)));

  // === Mobility ===
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(wifiApNode);
  mobility.Install(wifiStaNodes);

  // === IP stack and addressing ===
  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");

  Ipv4InterfaceContainer apInterface = address.Assign(apDevices);
  Ipv4InterfaceContainer staInterface = address.Assign(staDevices);

  PopulateARPcache();

  phy.EnablePcapAll("wifi-stats");

    // Install applications (traffic generators)
    ApplicationContainer sourceApplications, sinkApplications;
    const uint16_t portNumber = 9;

    // Sink on AP
    auto apIpv4 = wifiApNode.Get(0)->GetObject<Ipv4>();
    Ipv4Address apAddr = apIpv4->GetAddress(1, 0).GetLocal();
    InetSocketAddress sinkSocket(apAddr, portNumber);

    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkSocket);
    sinkApplications.Add(packetSinkHelper.Install(wifiApNode.Get(0)));

    // Client only on the first station: 1 packet
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

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",
                  MakeCallback(&PhyRxEndHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
                MakeCallback(&PerDevicePhyStateTracker));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                MakeCallback(&MacTxSuccessPerDev));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                MakeCallback(&MacTxFailPerDev));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                MakeCallback(&CountTxPackets));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                MakeCallback(&CountRxPackets));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                MakeCallback(&DetailedPhyRxDropHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxEnd",
                MakeCallback(&PhyTxEndHandler));



  // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
  //                 MakeCallback(&MacTxOkHandler));

  // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
  //                 MakeCallback(&MacTxFailedHandler));

  // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
  //                 MakeCallback(&MacRetryHandler));

    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
  //                 MakeCallback(&PhyRxDropHandler));


  // Run the simulation!
  Simulator::Run();

  // Record stop time and count duration
  auto finish = std::chrono::high_resolution_clock::now();
  std::clog << ("done!") << std::endl;
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n\n";


////////////////////////////////////////
// ============================================================================
// === RESULTS SUMMARY ===
double throughput = 0;
for (uint32_t index = 0; index < sinkApplications.GetN(); ++index)
{
    uint64_t totalBytesThrough = DynamicCast<PacketSink>(sinkApplications.Get(index))->GetTotalRx();
    throughput += ((totalBytesThrough * 8) / (simulationTime * 1000000.0)); // Mbit/s
}

std::cout << "Results:\n";
std::cout << "- aggregate throughput: " << throughput << " Mbit/s\n";

// ============================================================================
// === PHY LAYER STATISTICS ===
// ============================================================================

std::cout << "\n=== PHY LAYER STATISTICS ===\n";

// --- Per-node/device time share ---
std::cout << "\n--- Time Share per Node/Device ---\n";
for (const auto& kv : gTxTimePerDev)
{
    auto key = kv.first;
    double tx  = gTxTimePerDev[key];
    double rx  = gRxTimePerDev[key];
    double cca = gCcaTimePerDev[key];
    double idle = gIdleTimePerDev[key];


    std::cout << "Node " << key.first << " | Dev " << key.second << "\n";
    std::cout << "IDLE:      " << std::fixed << std::setprecision(6) << idle << " s\n";
    std::cout << "CCA_BUSY:  " << cca << " s\n";
    std::cout << "TX:        " << tx  << " s\n";
    std::cout << "RX:        " << rx  << " s\n";
    std::cout << "OTHER:     0 s\n\n";
}

// --- Channel Utilization ---
std::cout << "--- Channel Utilization per Node/Device ---\n";
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

// --- Tx Opportunity Duration ---
std::cout << "\n--- Tx Opportunity Duration ---\n";
for (const auto& kv : gTxOppDurationPerDev)
{
  auto key = kv.first;
  double txOpp = kv.second;
  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | Tx Opportunity Duration = "
            << std::fixed << std::setprecision(6) << txOpp << " s\n";
}

// --- PSDU-Level PHY Counters ---
std::cout << "\n--- PHY PSDU Statistics ---\n"
          << "PHY header failures: " << gPhyHeaderFailures << "\n"
          << "RX while decoding preamble: " << gRxWhileDecoding << "\n"
          << "RX aborted by TX: " << gRxAbortedByTx << "\n"
          << "PSDU successes: " << gPsduSuccesses << "\n"
          << "PSDU failures:  " << gPsduFailures << "\n"
          << "Total RX events: " << gTotalRxEvents << "\n"
          << "Total PHY events: " << gTotalPhyEvents << "\n";

// --- PSDU Success per Receiver ---
std::cout << "\n--- PSDU Success per Receiver ---\n";
for (const auto& kv : gPsduSuccessPerDev)
{
    std::cout << "Node " << kv.first.first
              << " | Dev " << kv.first.second
              << " | PSDU Success = " << kv.second << std::endl;
}

// --- Total RX events per receiver (PSDU-based) ---
std::cout << "\n--- Total RX events per receiver (PSDU-based) ---\n";

uint32_t totalPsduSuccess = 0;
for (const auto& entry : gPsduSuccessPerDev)
{
    totalPsduSuccess += entry.second;
}

std::cout << "Total RX events per receiver (PSDU-based): "
          << totalPsduSuccess << std::endl;

// --- PHY Frame Classification ---
std::cout << "\n--- Dropped frame types (PHY RX drop classification) ---" << std::endl;
std::cout << "Dropped DATA frames:      " << gDroppedDataFrames << std::endl;
std::cout << "Dropped ACK frames:       " << gDroppedAckFrames << std::endl;
std::cout << "Dropped RTS frames:       " << gDroppedRtsFrames << std::endl;
std::cout << "Dropped CTS frames:       " << gDroppedCtsFrames << std::endl;
std::cout << "Dropped BlockAck frames:  " << gDroppedBlockAckFrames << std::endl;

std::cout << "\n--- Received frame types (PHY RX end classification) ---" << std::endl;
std::cout << "Received DATA frames:      " << gTotalDataFrames << std::endl;
std::cout << "Received ACK frames:       " << gTotalAckFrames << std::endl;
std::cout << "Received RTS frames:       " << gTotalRtsFrames << std::endl;
std::cout << "Received CTS frames:       " << gTotalCtsFrames << std::endl;
std::cout << "Received BlockAck frames:  " << gTotalBlockAckFrames << std::endl;

std::cout << "\n--- Transmitted frame types (PHY TX classification) ---" << std::endl;
std::cout << "Transmitted DATA frames:      " << gTotalTxDataFrames << std::endl;
std::cout << "Transmitted ACK frames:       " << gTotalTxAckFrames << std::endl;
std::cout << "Transmitted RTS frames:       " << gTotalTxRtsFrames << std::endl;
std::cout << "Transmitted CTS frames:       " << gTotalTxCtsFrames << std::endl;
std::cout << "Transmitted BlockAck frames:  " << gTotalTxBlockAckFrames << std::endl;


// ============================================================================
// === MAC LAYER STATISTICS ===
// ============================================================================
std::cout << "\n=== MAC LAYER STATISTICS ===\n";

// --- Global MPDU counters ---
uint64_t mpduSuccesses = txStats.GetSuccesses();
uint64_t mpduFailures = txStats.GetFailures();
uint64_t mpduRetries = txStats.GetRetransmissions();

std::cout << "MPDU Successes:   " << mpduSuccesses << "\n"
          << "MPDU Failures:    " << mpduFailures  << "\n"
          << "MPDU Retransmits: " << mpduRetries   << "\n";


// --- Per-node/device MPDU breakdown ---
std::cout << "\n--- MPDU per Node/Device ---\n";
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

// --- Aggregate MAC transmission statistics ---
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

std::cout << "Tx Attempts (aggregate): " << totalTxAttempts << "\n"
          << "Tx Retries (aggregate):  " << totalRetries << "\n"
          << "Tx Failures (aggregate): " << totalFailures << "\n";

// --- Derived performance metrics per node/device ---
std::cout << "\n--- MAC Derived Performance Metrics ---\n";
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

// --- MAC Throughput per node/device ---
std::cout << "\n--- MAC Throughput per Node/Device ---\n";
for (const auto &kv : gMpduTxPerDev)
{
  auto key = kv.first;
  uint64_t txPackets = kv.second;
  double throughputMbps = 0.0;

  if (gSimulationTime > 0)
  {
    throughputMbps = (txPackets * gPacketSize * 8.0) / (gSimulationTime * 1e6);
  }

  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | TX Packets=" << txPackets
            << " | Throughput=" << std::fixed << std::setprecision(6)
            << throughputMbps << " Mbit/s\n";
}

// ============================================================================
// === NETWORK & APPLICATION LAYER ===
// ============================================================================
std::cout << "\n=== NETWORK & APPLICATION LAYER STATISTICS ===\n";

// --- TX/RX Packets and Bytes ---
std::cout << "\n--- Packets and Bytes per Node/Device ---\n";
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

// --- Per-Sender Summary ---
std::cout << "\n--- Per Sender ---\n";
for (const auto& entry : gTxPacketsPerDev) {
    const auto& key = entry.first;
    uint64_t sent = entry.second;
    std::cout << "Node " << key.first << " | Dev " << key.second
              << " | Sent=" << sent << "\n";
}

// --- Per-Receiver Summary ---
std::cout << "\n--- Per Receiver ---\n";
for (const auto& entry : gRxPacketsPerDev) {
    const auto& key = entry.first;
    uint64_t received = entry.second;
    std::cout << "Node " << key.first << " | Dev " << key.second
              << " | Received=" << received << "\n";

    if (gFirstRxTimePerDev.count(key) && gLastRxTimePerDev.count(key)) {
        double firstRx = gFirstRxTimePerDev.at(key).GetSeconds();
        double lastRx  = gLastRxTimePerDev.at(key).GetSeconds();
        double duration = std::max(0.0, lastRx - firstRx);
        std::cout << "   ↳ First RX=" << firstRx
                  << " s, Last RX=" << lastRx
                  << " s, Data transfer duration=" << duration << " s\n";
    }
}

// --- Aggregate Packet Statistics ---
uint64_t totalSent = 0, totalReceived = 0;
for (const auto& e : gTxPacketsPerDev) totalSent += e.second;
for (const auto& e : gRxPacketsPerDev) totalReceived += e.second;

uint64_t totalLost = (totalSent > totalReceived) ? (totalSent - totalReceived) : 0;
double totalLossRatio = (totalSent > 0)
    ? (static_cast<double>(totalLost) / totalSent) * 100.0
    : 0.0;

std::cout << "\n--- Aggregate Packet Statistics ---\n"
          << "Sent=" << totalSent
          << ", Received=" << totalReceived
          << ", Lost=" << totalLost
          << ", Loss Ratio=" << std::fixed << std::setprecision(2)
          << totalLossRatio << "%\n";

// uint64_t totalLost2 = (totalSent > totalReceived) ? (totalSent - totalReceived) : 0;
// double totalLossRatio2 = (totalSent > 0)
//     ? (static_cast<double>(totalLost2) / totalSent) * 100.0
//     : 0.0;

// std::cout << "\n--- Aggregate Packet Statistics ---\n"
//           << "Sent=" << totalSent
//           << ", Received=" << totalReceived
//           << ", Lost=" << totalLost2
//           << ", Loss Ratio=" << std::fixed << std::setprecision(2)
//           << totalLossRatio2 << "%\n";


// --- RX Timing per Node/Device ---
std::cout << "\n--- RX Timing per Node/Device ---\n";
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

// --- Receiver-side Throughput ---
std::cout << "\n--- Receiver-side Throughput ---\n";
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

// --- Delay and Jitter Statistics ---
std::cout << "\n--- Delay and Jitter ---\n";

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

// ============================================================================
// === BLOCK ACK & ADVANCED PHY/MAC METRICS ===
// ============================================================================
std::cout << "\n=== BLOCK ACK & ADVANCED PHY/MAC METRICS ===\n";

// --- Block ACK Summary ---
uint64_t totalBlockAckRx = 0;
uint64_t totalBlockAckReqRx = 0;
for (const auto& [_, count] : gBlockAckRx)     totalBlockAckRx     += count;
for (const auto& [_, count] : gBlockAckReqRx)  totalBlockAckReqRx  += count;

std::cout << "\n--- Block ACK Summary ---\n";
std::cout << "Total BlockAck RX:    " << totalBlockAckRx << "\n";
std::cout << "Total BlockAckReq RX: " << totalBlockAckReqRx << "\n";

// --- Detailed Block ACK Events (per MAC address) ---
std::cout << "\n--- Detailed Block ACK Events (per MAC) ---\n";
if (gBlockAckRx.empty() && gBlockAckReqRx.empty())
{
    std::cout << "(no Block ACK events recorded)\n";
}
else
{
    for (const auto& [mac, count] : gBlockAckRx)
        std::cout << "MAC " << mac << " | BlockAck RX = " << count << "\n";
    for (const auto& [mac, count] : gBlockAckReqRx)
        std::cout << "MAC " << mac << " | BlockAckReq RX = " << count << "\n";
}

// --- PHY Error / Abort Events ---
std::cout << "\n--- PHY Error & Abort Events (per MAC) ---\n";
if (gPhyHeaderFailed.empty() && gRxEventWhileDecoding.empty() && gRxEventAbortedByTx.empty())
{
    std::cout << "(no PHY error/abort events recorded)\n";
}
else
{
    for (const auto& [mac, count] : gPhyHeaderFailed)
        std::cout << "MAC " << mac << " | PHY Header Failures = " << count << "\n";
    for (const auto& [mac, count] : gRxEventWhileDecoding)
        std::cout << "MAC " << mac << " | RX While Decoding = " << count << "\n";
    for (const auto& [mac, count] : gRxEventAbortedByTx)
        std::cout << "MAC " << mac << " | RX Aborted By TX = " << count << "\n";
}

// --- Aggregate Data Transfer Duration ---
double totalDataTransferDuration = 0.0;
for (const auto& [key, firstTime] : gFirstRxTimePerDev)
{
    if (gLastRxTimePerDev.count(key))
    {
        double duration = (gLastRxTimePerDev.at(key) - firstTime).GetSeconds();
        if (duration > 0.0)
            totalDataTransferDuration += duration;
    }
}

std::cout << "\n--- Aggregate Data Transfer Duration ---\n";
std::cout << "Total duration (sum over all receivers): "
          << std::fixed << std::setprecision(6)
          << totalDataTransferDuration << " s\n";



// ============================================================================
// === CLEAN-UP ===
Simulator::Destroy();
return 0;

}
