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


static std::pair<uint32_t, uint32_t> ParseNodeDevFromContext(const std::string& ctx);

static double gPhyIdleSec = 0.0, gPhyCcaSec = 0.0, gPhyTxSec = 0.0, gPhyRxSec = 0.0, gPhyOtherSec = 0.0;

// --- PSDU callbacks (success & failure) ---

// Poprawnie odebrany PSDU
static void
PhyRxEndHandler(std::string /*context*/, Ptr<const Packet> /*packet*/)
{
  gPsduSuccesses++;
  gTotalRxEvents++;
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

    // --- Odbiór przerwany przez rozpoczęcie transmisji TX ---
    case WifiPhyRxfailureReason::RECEPTION_ABORTED_BY_TX:
      gRxAbortedByTx++;
      break;



    // --- Faktyczne PSDU-level failures ---
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

  // Tworzymy wpisy ARP dla każdego interfejsu
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
              entry->MarkPermanent (); // wpis stały
            }
        }
    }

  // Przypisz cache do wszystkich interfejsów
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

    // --- Sink na AP ---
    auto apIpv4 = wifiApNode.Get(0)->GetObject<Ipv4>();
    Ipv4Address apAddr = apIpv4->GetAddress(1, 0).GetLocal();
    InetSocketAddress sinkSocket(apAddr, portNumber);

    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkSocket);
    sinkApplications.Add(packetSinkHelper.Install(wifiApNode.Get(0)));

    // --- Klient tylko na pierwszej stacji: 1 pakiet ---
    UdpClientHelper client(sinkSocket);
    client.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    client.SetAttribute("Interval", TimeValue(Seconds(interval)));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));


    sourceApplications.Add(client.Install(wifiStaNodes.Get(0)));

    // --- Timingi ---
    sinkApplications.Start(Seconds(0.0));
    sinkApplications.Stop(Seconds(simulationTime + 1));
    sourceApplications.Start(Seconds(1.0));
    sourceApplications.Stop(Seconds(simulationTime + 1));


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

  Config::ConnectWithoutContext(
  "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
  MakeCallback(&PhyStateLogger)
  );

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

std::cout << "\n--- Channel Utilization per node/device ---\n";
for (const auto& kv : gTxTimePerDev)
{
  auto key = kv.first;
  double tx = kv.second;
  double rx = gRxTimePerDev[key];
  double cca = gCcaTimePerDev[key];
  double busy = tx + rx + cca;
  double totalSimTime = simulationTime + 1.0; // tak jak w Twoim Simulator::Stop
  double utilPct = (busy / totalSimTime) * 100.0;

  std::cout << "Node " << key.first
            << " | Dev " << key.second
            << " | TX=" << std::fixed << std::setprecision(6) << tx
            << " s, RX=" << rx
            << " s, CCA=" << cca
            << " s | Utilization=" << std::setprecision(2)
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



  // Clean-up
  Simulator::Destroy();

  return 0;
}
