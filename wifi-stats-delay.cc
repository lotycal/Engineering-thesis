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
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/wifi-phy.h"
#include <iomanip>





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

// --- PHY (PSDU) counters ---
static uint64_t gPsduSuccesses = 0;
static uint64_t gPsduFailures  = 0;

static std::map<uint64_t, Time> gTxTimestampPerSeq;
static std::vector<double> gDelays;

static void PhyRxEndHandler(std::string /*context*/, Ptr<const Packet> /*p*/)
{
    gPsduSuccesses++;
}

static void PhyRxDropHandler(std::string /*context*/,
                             Ptr<const Packet> /*p*/,
                             WifiPhyRxfailureReason /*reason*/)
{
    gPsduFailures++;
}

static void TxTrace(std::string context, Ptr<const Packet> packet)
{
    gTxTimestampPerSeq[packet->GetUid()] = Simulator::Now();
}

static void RxTrace(std::string context, Ptr<const Packet> packet)
{
    uint64_t id = packet->GetUid();
    auto it = gTxTimestampPerSeq.find(id);

    if (it != gTxTimestampPerSeq.end())
    {
        double delay = (Simulator::Now() - it->second).GetSeconds();
        gDelays.push_back(delay);
        gTxTimestampPerSeq.erase(it);
    }
}


int main(int argc, char *argv[])
{

  // Initialize default simulation parameters
  uint32_t nWifi = 1;        // Number of transmitting stations
  double simulationTime = 1; // Default simulation time [s]
  int mcs = 11;              // Default MCS is set to highest value
  int channelWidth = 20;     // Default channel width [MHz]
  int gi = 800;              // Default guard interval [ns]

  // Parse command line arguments
  CommandLine cmd;
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue("mcs", "use a specific MCS (0-11)", mcs);
  cmd.AddValue("nWifi", "number of stations", nWifi);
  cmd.Parse(argc, argv);

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
  wifi.SetStandard(WIFI_STANDARD_80211ax);

  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
  
  // Set channel width for given PHY
  std::string channelStr("{0, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}");
  phy.Set("ChannelSettings", StringValue(channelStr));

  std::ostringstream oss;
  oss << "HeMcs" << mcs;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
                               "ControlMode", StringValue(oss.str())); // Set MCS

  Ssid ssid = Ssid("ns3-80211ax"); // Set SSID

  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid));

  // Create and configure Wi-Fi interfaces
  NetDeviceContainer staDevice;
  staDevice = wifi.Install(phy, mac, wifiStaNodes);

  mac.SetType("ns3::ApWifiMac",
              "Ssid", SsidValue(ssid));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install(phy, mac, wifiApNode);

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

  // WifiTxStatsHelper txStats;
  // txStats.Enable(apDevice);
  // txStats.Enable(staDevice);

  // Configure IP addressing
  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterface = address.Assign(staDevice);
  apNodeInterface = address.Assign(apDevice);

  // Install applications (traffic generators)
  ApplicationContainer sourceApplications, sinkApplications;
  uint32_t portNumber = 9;
  for (uint32_t index = 0; index < nWifi; ++index) // Loop over all stations (which transmit to the AP)
  {
    auto ipv4 = wifiApNode.Get(0)->GetObject<Ipv4>();                       // Get destination's IP interface
    const auto address = ipv4->GetAddress(1, 0).GetLocal();                 // Get destination's IP address
    InetSocketAddress sinkSocket(address, portNumber++);                    // Configure destination socket
    OnOffHelper onOffHelper("ns3::UdpSocketFactory", sinkSocket);
    onOffHelper.SetConstantRate(DataRate(150e6 / nWifi), 1472);
    sourceApplications.Add(onOffHelper.Install(wifiStaNodes.Get(index)));
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkSocket); // Configure traffic sink
    sinkApplications.Add(packetSinkHelper.Install(wifiApNode.Get(0)));      // Install traffic sink
  }

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

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",
                MakeCallback(&PhyRxEndHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                MakeCallback(&PhyRxDropHandler));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                MakeCallback(&TxTrace));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                MakeCallback(&RxTrace));

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

  const uint64_t totalPsdus = gPsduSuccesses + gPsduFailures;
  double pSucc = 0.0;
  double pColl = 0.0;

  if (totalPsdus > 0) {
    pSucc = static_cast<double>(gPsduSuccesses) / static_cast<double>(totalPsdus);
    pColl = static_cast<double>(gPsduFailures)  / static_cast<double>(totalPsdus); // <-- jak wcześniej
  }

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "\n=== PHY LAYER (PSDU) ===\n";
  std::cout << "PSDU Successes: " << gPsduSuccesses << "\n";
  std::cout << "PSDU Failures:  " << gPsduFailures  << "\n";
  std::cout << "P(success) = " << pSucc << " (" << (pSucc * 100.0) << "%)\n";
  std::cout << "Estimated collision probability (PSDU) = "
            << pColl << " (" << (pColl * 100.0) << "%)\n";

  std::ofstream csv("delays.csv");
  csv << "delay_s,delay_us\n";
  for (double d : gDelays)
  {
      csv << d << "," << d * 1e6 << "\n";
  }
  csv.close();

  std::vector<double> sorted = gDelays;
  std::sort(sorted.begin(), sorted.end());
  size_t N = sorted.size();

  std::ofstream cdf("delays_cdf.csv");
  cdf << "delay_s,cdf\n";

  for (size_t i = 0; i < N; ++i)
  {
      double Fx = double(i + 1) / N;
      cdf << sorted[i] << "," << Fx << "\n";
  }
  cdf.close();




  // Clean-up
  Simulator::Destroy();

  return 0;
}
